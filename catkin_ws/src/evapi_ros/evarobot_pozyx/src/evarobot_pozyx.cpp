#include "evarobot_pozyx/evarobot_pozyx.h"

/**
 * Merges raw data into merged_data in type of int32_t.
 */
void mergeData(char * raw_data, uint8_t data_length, int32_t & merged_data) {
	merged_data = 0;
	for (int i = 0; i < data_length; i++) {
		merged_data |= raw_data[i] << ((data_length-1-i)*8);
	}
	return;
}

/**
 * Merges raw data into merged_data in type of int16_t.
 */
void mergeData(char * raw_data, uint8_t data_length, int16_t & merged_data) {
	merged_data = 0;
	for (int i = 0; i < data_length; i++) {
		merged_data |= raw_data[i] << ((data_length-1-i)*8);
	}
	return;
}

/**
 * Merges raw data into merged_data in type of int32_t.
 */
void mergeData(char * raw_data, uint8_t data_length, int32_t * merged_data) {
	int mod_no = 4;
	int j = -1;

	for (int i = 0; i < data_length; i++) {
		if ( (i % mod_no) == 0 ) {
			++j;
			merged_data[j] = 0;
		}
		merged_data[j] |= raw_data[i] << ( (4-1-(i%mod_no) )*8);
	}

	return;
}

/**
 * Merges raw data into merged_data in type of int16_t.
 */
void mergeData(char * raw_data, uint8_t data_length, int16_t * merged_data) {
	int mod_no = 2;
	int j = -1;

	for (int i = 0; i < data_length; i++) {
		if ( (i % mod_no) == 0 ) {
			++j;
			merged_data[j] = 0;
		}
		merged_data[j] |= raw_data[i] << ( (2-1-(i%mod_no) )*8);
	}

	return;
}

/**
 * Parses serial data.
 */
int8_t parseSerialData(list<char> & rx_data, PACKET_STRUCT * packet) {

	uint8_t index = 0;
	bool isStartFlag = false;
	list<char>::iterator it_packet, it;

	/**
	 * Find start flag which is 'ino'
	 */
	for ( it = rx_data.begin(); index < rx_data.size(); it++) {
		if(*it == 'i') {
			advance(it, 1);
			if ( *it == 'n' ) {
				advance(it, 1);
				if ( *it == 'o' ) {
					isStartFlag = true;
					advance(it, -2);
					it_packet = it;
					break;
				}
			}
		}
		++index;
	}

	uint8_t packet_index = index;

	/**
	 * Found packet header ?
	 */
	if ( !isStartFlag ) {
		return -2;
	}

	/**
	 * Check if left enough room
	 */
	if ( (rx_data.size() - packet_index) < 7 ) {
		return -3;
	}

	it = it_packet;
	advance(it, 3);
	char packet_type = *it;

	char packet_has_data = (packet_type >> 7) & 0x01;
	uint8_t batch_length = (packet_type >> 3) & 0x0F;
	uint8_t multiplier = (packet_type >> 1) & 0x03;

	uint8_t data_length = 0;

	if ( packet_has_data ) {
		data_length = pow(2, multiplier) * batch_length;
	} else {
		data_length = 0;
	}

	/**
	 * Check full packet
	 */
	if ( (rx_data.size() - packet_index) < (data_length + 5) ) {
		return -3;
	}

	it = it_packet;
	advance(it, 4);
	packet->address = *(it);
	packet->packet_type = packet_type;
	packet->data_length = data_length;

	/**
	 * Calculate checksum
	 */
	uint16_t computed_checksum = 'i' + 'n' + 'o';
	computed_checksum += packet->packet_type;
	computed_checksum += packet->address;

	#ifdef DEBUG
	printf("packet type: %x \n", packet->packet_type);
	printf("data length: %d \n", packet->data_length);
	printf("address: %x \n", packet->address);
	#endif

	it = it_packet;
	advance(it, 5);
	for ( index = 0; index < data_length; index++ ) {
		packet->data[index] = *(it);
		it++;
		computed_checksum += packet->data[index];

		#ifdef DEBUG
		printf("DATA[%d]: %x \n", index, packet->data[index]);
		#endif
	}

	/**
	 * Received checksum
	 */
	it = it_packet;
	advance(it, 5 + data_length);
	char received_checksum_H = *(it);
	it++;
	char received_checksum_L = *(it);
	uint16_t received_checksum = (received_checksum_H << 8) | received_checksum_L;

	#ifdef DEBUG
	printf("Received: %x Computed: %x \n", received_checksum, computed_checksum);
	#endif

	// Compare checksums
	if ( received_checksum != computed_checksum)
		return -4;

	it = it_packet;
	advance(it, (7 + data_length));
	rx_data.erase(rx_data.begin(), it);

	return 0;
}

inline float rejectOutliers(vector<float> data, int dataSize, int frameSize) {
	float avg = 0, avg_dummy = 0;;
	for ( int i = 0; i < dataSize; i++ ) {
	float sum = 0;
	for ( int j = i; j < i+frameSize; j++) {
	sum += data[i%dataSize];
	}
	avg_dummy = sum / frameSize;
	if ( i == 0 || avg > avg_dummy ) {
	  avg = avg_dummy;
	}
	}
	return avg;
}

/**
 * Program starts here
 */
int main(int argc, char **argv) {
	// ROS PARAMS
	string str_device_id;
	//  int i_data_size, i_frame_size;

	/**
	 * Initializes ROS node with evarobot_pozyx name.
	 */
	ros::init(argc, argv, "evarobot_pozyx");

	/**
	 * Creates ROS node handler.
	 */
	ros::NodeHandle n;

	/**
	 * Publisher topic is created with pozyx topic name and nav_msgs::Odometry message type.
	 */
	ros::Publisher pozyx_pub = n.advertise<nav_msgs::Odometry>("pozyx", 1);

	/**
	 * Define frequency
	 */
	ros::Rate loop_rate(100.0);

	/**
	 * Create object from IMSerial to communication with pozyx.
	 */
	IMSerial serial("/dev/ttyACM0", B115200, CS8, 0, 0, 0);
	char rx_data[200];
	uint8_t rx_length = 0;

	PACKET_STRUCT POZYX_PACKET;

	/**
	 * ROS message to send pozyx topic.
	 */
	nav_msgs::Odometry msg;
	int8_t ret_val = 0;

	/**
	 * Calculated x, y and z positions respectively.
	 */
	int32_t posX, posY, posZ, posAll[3];

	/**
	 * Calculated errors in x, y, z, xy, xz and yz axes respectively.
	 */
	int16_t posErrX, posErrY, posErrZ, posErrXY, posErrXZ, posErrYZ, posErrAll[6];
	int16_t quatAll[4];
	bool isReceivedPos = false, isReceivedPosErr = false, isReceivedQuat = false;
	list<char> rx_datalist;
	float pos_covariance[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

	int packet_count = 0;

	while(ros::ok()) {
	try {
		/**
		 * Read data from serial port.
		 */
		rx_length = serial.ReadData(rx_data, sizeof(rx_data));

		/**
		 * Convert data to list and parse.
		 */
		for ( int i = 0; i < rx_length; i++ ) {
			if ( rx_datalist.size() > RX_BUFFER_SIZE ) {
				ROS_INFO("Error: Serial RX buffer size is overloaded");
				return 0;
			} else {
				rx_datalist.push_back(rx_data[i]);
			}
		}
		ret_val = parseSerialData(rx_datalist, & POZYX_PACKET);

		#ifdef DEBUG
		printf("RetVal: %d \n", ret_val);
		#endif

		/**
		 * Calculate position using parsed data.
		 */
		if( ret_val == 0 ) {
			switch ( POZYX_PACKET.address ) {
				case POZYX_POS_X:
					mergeData(POZYX_PACKET.data, POZYX_PACKET.data_length, posX);
					#ifdef DEBUG
					printf("posX: %d mm\n", posX);
					#endif
					break;

				case POZYX_POS_Y:
					mergeData(POZYX_PACKET.data, POZYX_PACKET.data_length, posY);
					#ifdef DEBUG
					printf("posY: %d mm\n", posY);
					#endif
					break;

				case POZYX_POS_Z:
					mergeData(POZYX_PACKET.data, POZYX_PACKET.data_length, posZ);
					#ifdef DEBUG
					printf("posZ: %d mm\n", posZ);
					#endif
					break;

				case POZYX_POS_ERR_X:
					mergeData(POZYX_PACKET.data, POZYX_PACKET.data_length, posErrX);
					#ifdef DEBUG
					printf("posErrX: %d \n", posErrX);
					#endif
					break;

				case POZYX_POS_ERR_Y:
					mergeData(POZYX_PACKET.data, POZYX_PACKET.data_length, posErrY);
					#ifdef DEBUG
					printf("posErrY: %d \n", posErrY);
					#endif
					break;

				case POZYX_POS_ERR_Z:
					mergeData(POZYX_PACKET.data, POZYX_PACKET.data_length, posErrZ);
					#ifdef DEBUG
					printf("posErrZ: %d \n", posErrZ);
					#endif
					break;

				case POZYX_POS_ERR_XY:
					mergeData(POZYX_PACKET.data, POZYX_PACKET.data_length, posErrXY);
					#ifdef DEBUG
					printf("posErrXY: %d \n", posErrXY);
					#endif
					break;

				case POZYX_POS_ERR_XZ:
					mergeData(POZYX_PACKET.data, POZYX_PACKET.data_length, posErrXZ);
					#ifdef DEBUG
					printf("posErrXZ: %d \n", posErrXZ);
					#endif
					break;

				case POZYX_POS_ERR_YZ:
					mergeData(POZYX_PACKET.data, POZYX_PACKET.data_length, posErrYZ);
					#ifdef DEBUG
					printf("posErrYZ: %d \n", posErrYZ);
					#endif
					break;

				case POZYX_POS_ALL:
					mergeData(POZYX_PACKET.data, POZYX_PACKET.data_length, posAll);
					#ifdef DEBUG
					printf("posALL: x: %d, y: %d, z: %d \n", posAll[0], posAll[1], posAll[2]);
					#endif
					msg.header.stamp = ros::Time::now();
					isReceivedPos = true;
					break;

				case POZYX_POS_ERR_ALL:
					mergeData(POZYX_PACKET.data, POZYX_PACKET.data_length, posErrAll);
					#ifdef DEBUG
					printf("posErrALL: x: %d, y: %d, z: %d, xy: %d, xz: %d, yz: %d \n", posErrAll[0], posErrAll[1], posErrAll[2], posErrAll[3], posErrAll[4], posErrAll[5]);
					#endif
					isReceivedPosErr = true;
					break;

				case POZYX_QUAT_ALL:
					mergeData(POZYX_PACKET.data, POZYX_PACKET.data_length, quatAll);
					#ifdef DEBUG
					printf("quatALL: x: %d, y: %d, z: %d, w: %d \n", quatAll[0], quatAll[1], quatAll[2], quatAll[3]);
					#endif
					isReceivedQuat = true;
			}

			/**
			 * Clear data
			 */
			  memset(POZYX_PACKET.data, 0, sizeof(POZYX_PACKET.data));

			  if ( isReceivedPos && isReceivedPosErr && isReceivedQuat) {
				isReceivedPos = false;
				isReceivedPosErr = false;
				isReceivedQuat = false;

				/**
				 * Set ROS message content.
				 */
				msg.header.frame_id = "map";
				msg.child_frame_id = "odom";
				msg.pose.pose.position.x = (float)(posAll[0] * 0.001); //  rejectOutliers(T_posX, i_data_size, i_frame_size);
				msg.pose.pose.position.y = (float)(posAll[1] * 0.001); //  rejectOutliers(T_posY, i_data_size, i_frame_size);
				msg.pose.pose.position.z = (float)(posAll[2] * 0.001); //  rejectOutliers(T_posZ, i_data_size, i_frame_size);

				msg.pose.pose.orientation.x = (float)(quatAll[0] * 0.001); // T_oriX[9]; //  rejectOutliers(T_oriX, i_data_size, i_frame_size);
				msg.pose.pose.orientation.y = (float)(quatAll[1] * 0.001); // T_oriY[9]; //  rejectOutliers(T_oriY, i_data_size, i_frame_size);
				msg.pose.pose.orientation.z = (float)(quatAll[2] * 0.001); // T_oriZ[9]; //  rejectOutliers(T_oriZ, i_data_size, i_frame_size);
				msg.pose.pose.orientation.w = (float)(quatAll[3] * 0.001); // T_oriW[9]; //  rejectOutliers(T_oriW, i_data_size, i_frame_size);

			  
				for ( int i = 0; i < 6; i++ ) {
				  pos_covariance[i] = 9999; // posErrAll[i] * 0.001;
				  if ( pos_covariance[i] == 0.0 ) {
				pos_covariance[i] = 1.0;
				  }
				}

				float covariance[36] =	{pos_covariance[0], 0, 0, 0, 0, 0,  // covariance on gps_x
							 0, pos_covariance[1], 0, 0, 0, 0,  // covariance on gps_y
							 0, 0, pos_covariance[2], 0, 0, 0,  // covariance on gps_z
							 0, 0, 0, pos_covariance[5], 0, 0,  // large covariance on rot x				 0, 0, 0, 0, pos_covariance[4], 0,  // large covariance on rot y
							 0, 0, 0, 0, 0, pos_covariance[3]};  // large covariance on rot z	

				for ( int i = 0; i < 36; i++) {
					msg.pose.covariance[i] = covariance[i];
				}

				/**
				 * Publish message from pozyx topic.
				 */
				pozyx_pub.publish(msg);
			}
		  } else {
			#ifdef DEBUG
			for (uint8_t i = 0; i < rx_length; i++)
			  printf("0x%x,", rx_data[i]);
			printf("\n");
			#endif
		  }
	} catch(int e) {
	  ROS_INFO(GetErrorDescription(e).c_str());
	}
		/**
		 * Clear data.
		 */
		memset(rx_data, 0, sizeof(rx_data));

		/**
		 * Loop is slept to hold frequency.
		 */
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
