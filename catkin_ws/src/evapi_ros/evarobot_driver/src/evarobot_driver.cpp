#include "evarobot_driver/evarobot_driver.h"

/**
 * Constructor with custom parameters.
 */
IMDRIVER::IMDRIVER(double d_limit_voltage,
                   float f_max_lin_vel,
                   float f_max_ang_vel,
                   float f_wheel_separation,
                   float f_wheel_diameter,
                   double d_frequency,
                   unsigned int u_i_counts,
                   double d_duty,
                   int i_mode,
                   boost::shared_ptr< IMGPIO > * m1_in,
                   boost::shared_ptr< IMGPIO > * m2_in,
                   IMGPIO * m1_en,
                   IMGPIO * m2_en,
                   IMADC * adc,
                   ros::ServiceClient & client,
                   double _timeout):b_left_motor_error(false), b_right_motor_error(false), d_timeout(_timeout), b_motor_error(false)
{
    this->client = client;
    this->adc = adc;
    this->d_limit_voltage = d_limit_voltage;

    this->f_left_motor_voltage = 0.0;
    this->f_right_motor_voltage = 0.0;

	this->u_i_counts = u_i_counts;
	
	/**
	 * PWM controller to control motors.
	 */
	this->pwm = new IMPWM(d_frequency, u_i_counts, d_duty, i_mode);

	this->m1_in = m1_in;
	this->m2_in = m2_in;
	this->m1_en = m1_en;
	this->m2_en = m2_en;

	/**
	 * Sets pin direction as output.
	 */
    for(int i = 0; i < 2; i++)
        this->m1_in[i]->SetPinDirection(IMGPIO::OUTPUT);

    for(int i = 0; i < 2; i++)
        this->m2_in[i]->SetPinDirection(IMGPIO::OUTPUT);

    /**
     * Turns on leds.
     */
    im_msgs::SetRGB srv;

    srv.request.times = -1;
    srv.request.mode = 0;
    srv.request.frequency = 1.0;
    srv.request.color = 0;

    if(this->client.call(srv) == 0)
    {
        ROS_INFO(GetErrorDescription(-77).c_str());
        i_error_code = -77;
    }

	/**
	 * Enables motors.
	 */
    this->Enable();

}

/**
 * Destructor clears data and turns off leds.
 */
IMDRIVER::~IMDRIVER()
{
    im_msgs::SetRGB srv;

    srv.request.times = -1;
    srv.request.mode = 0;
    srv.request.frequency = 1.0;
    srv.request.color = 0;

    if(this->client.call(srv) == 0)
    {
        ROS_INFO(GetErrorDescription(-77).c_str());
        i_error_code = -77;
    }
	
    this->Disable();
    delete pwm;
    delete m1_en;
    delete m2_en;
    delete adc;
}

/**
 * Sets new parameters and prints information message at debug level.
 */
void IMDRIVER::UpdateParams()
{
    if(b_is_received_params)
    {
        ROS_DEBUG("EvarobotDriver: Updating Driver Params...");
        ROS_DEBUG("EvarobotDriver: %f, %f", g_d_max_lin, g_d_max_ang);
        ROS_DEBUG("EvarobotDriver: %f, %f", g_d_wheel_separation, g_d_wheel_diameter);

        this->f_max_lin_vel = g_d_max_lin;
        this->f_max_ang_vel = g_d_max_ang;
        this->f_wheel_separation = g_d_wheel_separation;
        this->f_wheel_diameter = g_d_wheel_diameter;

        b_is_received_params = false;
    }
}

/**
 * Applies input velocities to motors if velocities are in limits.
 */
void IMDRIVER::ApplyVel(float f_left_wheel_velocity, float f_right_wheel_velocity)
{
  	bool isStop = false;

	/**
	 * Set pin values according to f_left_wheel_velocity parameter.
	 */
    if(f_left_wheel_velocity > 0)
    {
        this->m1_in[0]->SetPinValue(IMGPIO::LOW);
        this->m1_in[1]->SetPinValue(IMGPIO::HIGH);
    }
    else if(f_left_wheel_velocity < 0)
    {
        this->m1_in[0]->SetPinValue(IMGPIO::HIGH);
        this->m1_in[1]->SetPinValue(IMGPIO::LOW);
    }
    else
    {
        this->m1_in[0]->SetPinValue(IMGPIO::LOW);
        this->m1_in[1]->SetPinValue(IMGPIO::LOW);
    }

    /**
	 * Set pin values according to f_right_wheel_velocity parameter.
	 */
    if(f_right_wheel_velocity > 0)
    {
        this->m2_in[0]->SetPinValue(IMGPIO::HIGH);
        this->m2_in[1]->SetPinValue(IMGPIO::LOW);
    }
    else if(f_right_wheel_velocity < 0)
    {
        this->m2_in[0]->SetPinValue(IMGPIO::LOW);
        this->m2_in[1]->SetPinValue(IMGPIO::HIGH);
    }
    else
    {
        this->m2_in[0]->SetPinValue(IMGPIO::LOW);
        this->m2_in[1]->SetPinValue(IMGPIO::LOW);
		isStop = true;
    }

    /**
     * Compute left and right wheel duty cycles.
     */
    int i_left_wheel_duty = int(fabs(f_left_wheel_velocity) * 255 / this->f_max_lin_vel);
    int i_right_wheel_duty = int(fabs(f_right_wheel_velocity) * 255 / this->f_max_lin_vel);

    i_left_wheel_duty = i_left_wheel_duty>=this->u_i_counts ? this->u_i_counts - 1:i_left_wheel_duty;
    i_right_wheel_duty = i_right_wheel_duty>=this->u_i_counts ? this->u_i_counts - 1:i_right_wheel_duty;

    this->leftPWM = i_left_wheel_duty;
    this->rightPWM = i_right_wheel_duty;

    /**
     * Apply calculated duty cycles.
     */
    if ( isStop ) {
      pwm->SetDutyCycleCount(0, 0);
      pwm->SetDutyCycleCount(0, 1);
    } else {

      pwm->SetDutyCycleCount(i_left_wheel_duty, 0);
      pwm->SetDutyCycleCount(i_right_wheel_duty, 1);
    }


    /*	if(this->CheckError() == -1)
    	{
    		ROS_ERROR("Failure at Left Motor.");
    	}
    	else if(this->CheckError() == -2)
    	{
    		ROS_ERROR("Failure at Right Motor.");
    	}
    	else if(this->CheckError() == -3)
    	{
    		ROS_ERROR("Failure at Both of Motors.");
    	}*/
}

/**
 * Enables motor driver pins.
 */
void IMDRIVER::Enable()
{
    this->m1_en->SetPinDirection(IMGPIO::OUTPUT);
    this->m2_en->SetPinDirection(IMGPIO::OUTPUT);

    this->m1_en->SetPinValue(IMGPIO::HIGH);
    this->m2_en->SetPinValue(IMGPIO::HIGH);
}

/**
 * Disable motor driver pins.
 */
void IMDRIVER::Disable()
{
    this->m1_en->SetPinDirection(IMGPIO::OUTPUT);
    this->m2_en->SetPinDirection(IMGPIO::OUTPUT);

    this->m1_en->SetPinValue(IMGPIO::LOW);
    this->m2_en->SetPinValue(IMGPIO::LOW);
}

/**
 * Controls are there any error in related GPIO pins. 
 * If there is an error returns negative, else returns 0;
 */
int IMDRIVER::CheckError()
{
    int i_ret = 0;

    string str_m1_data;
    string str_m2_data;

    this->m1_en->GetPinValue(str_m1_data);
    this->m2_en->GetPinValue(str_m2_data);

    if(str_m1_data == IMGPIO::LOW)
    {
        --i_ret;
    }

    if(str_m2_data == IMGPIO::LOW)
    {
        --i_ret;
    }

    return i_ret;
}

/**
 * Returns left and right motor voltages.
 */
im_msgs::Voltage IMDRIVER::GetMotorVoltage() const
{
    im_msgs::Voltage ret;
    ret.left_motor = this->f_left_motor_voltage;
    ret.right_motor = this->f_right_motor_voltage;

    return ret;
}

/**
 * Controls motor current is in limits or not.
 */
bool IMDRIVER::CheckMotorCurrent()
{
    bool b_ret = true;

    /**
     * Calculetes current and controls value is in limits or not.
     */
    this->f_left_motor_voltage = (float)(this->adc->ReadMotorChannel(0)*5.0/4096.0);
    this->f_right_motor_voltage = (float)(this->adc->ReadMotorChannel(1)*5.0/4096.0);

    ROS_DEBUG("EvarobotDriver: LEFT MOTOR: -- %f", this->f_left_motor_voltage);
    ROS_DEBUG("EvarobotDriver: RIGHT MOTOR: -- %f", this->f_right_motor_voltage);

    if(this->f_left_motor_voltage >= this->d_limit_voltage)
    {
        ROS_INFO(GetErrorDescription(-78).c_str());
        i_error_code = -78;
        b_ret = false;
        this->b_motor_error = true;
        this->b_left_motor_error = true;
    }

    if(this->f_right_motor_voltage >= this->d_limit_voltage)
    {
        ROS_INFO(GetErrorDescription(-79).c_str());
        i_error_code = -79;
        b_ret = false;
        this->b_motor_error = true;
        this->b_right_motor_error = true;
    }

    /**
     * If there is an error, turns on leds ERROR mode.
     */
    if(!b_ret)
    {
        im_msgs::SetRGB srv;

        srv.request.times = -1;
        srv.request.mode = 8;
        srv.request.frequency = -1;
        srv.request.color = 0;

        if(this->client.call(srv) == 0)
        {
            ROS_INFO(GetErrorDescription(-77).c_str());
            i_error_code = -77;
        }

        this->Disable();
    }

    return b_ret;
}

/**
 *  If an error occurs publishes it.
 *  Else publishes both left and right motor voltages.
 */
void IMDRIVER::ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if(i_error_code < 0)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, GetErrorDescription(i_error_code).c_str());
        i_error_code = 0;
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motors are OK!");
    }

    stat.add("Right Motor Voltage", this->f_right_motor_voltage);
    stat.add("Left Motor Voltage", this->f_left_motor_voltage);

    stat.add("Right PWM",this->rightPWM);
    stat.add("Left PWM", this->leftPWM);

}

/**
 * Controls time difference between two consecutive data from cntr_wheel_vel topic.
 */
bool IMDRIVER::CheckTimeout()
{
	/**
	 * Time difference is calculated and controlled for limit overflow.
	 */
    double dt = (ros::Time::now() - this->curr_vel_time).toSec();

    if(dt > this->d_timeout)
    {
        ROS_INFO(GetErrorDescription(-80).c_str());
        i_error_code = -80;
        this->ApplyVel(0.0, 0.0);
        b_timeout_err = true;
    }
    else if(dt < 0)
    {
        ROS_INFO(GetErrorDescription(-81).c_str());
        i_error_code = -81;
        this->ApplyVel(0.0, 0.0);
        b_timeout_err = true;
    }
    else
    {
        b_timeout_err = false;
    }

    return this->b_timeout_err;
}

/**
 * Callback function for cntr_wheel_vel topic.
 */
void IMDRIVER::CallbackWheelVel(const im_msgs::WheelVel::ConstPtr & msg)
{
	/**
	 * Applies velocity according to input msg parameter.
	 */
    float f_left_wheel_velocity = msg->left_vel;
    float f_right_wheel_velocity = msg->right_vel;
    this->curr_vel_time = msg->header.stamp;
    this->UpdateParams();
    this->ApplyVel(f_left_wheel_velocity, f_right_wheel_velocity);
}

/**
 * Callback function to change pid parameters at runtime.
 */
void CallbackReconfigure(evarobot_driver::ParamsConfig &config, uint32_t level)
{
	b_is_received_params = true;
	g_d_max_lin = config.maxLinearVel;
    g_d_max_ang = config.maxAngularVel;
    g_d_wheel_separation = config.wheelSeparation;
    g_d_wheel_diameter = config.wheelDiameter;
}

/**
 * Program starts here.
 */
int main(int argc, char **argv)
{
	/**
	 * SPI mode between SPI_MODE_0 and SPI_MODE_3
	 */
	unsigned char u_c_spi_mode;
	
	/**
	 * Initializes ROS node with evarobot_driver name.
	 */
	ros::init(argc, argv, "/evarobot_driver");

	/**
	 * Creates ROS node handler.
	 */
	ros::NodeHandle n;

	/**
	 * SPI driver path.
	 */
	string str_driver_path;

	/**
	 * Maximum linear velocity.
	 */
	double d_max_lin_vel;
	/**
	 * Maximum angular velocity.
	 */
	double d_max_ang_vel;

	/**
	 * Diameter of wheels.
	 */
	double d_wheel_diameter;

	/**
	 * Separation between left and right wheels.
	 */
	double d_wheel_separation;

	/**
	 * PWM frequency.
	 */
	double d_frequency;
	/**
	 * PWM duty cycle.
	 */
	double d_duty;
	/**
	 * Voltage limit for both left and right wheels.
	 */
	double d_limit_voltage;

	/**
	 * Time limit between two consecutive data from cntr_wheel_vel topic.
	 */
	double d_timeout;

	/**
	 * PWM resolution.
	 */
	int i_counts;

	/**
	 * PWM mode (IMPWM::MSMODE or IMPWM::PWMMODE)
	 */
	int i_mode;

	/**
	 * Motor input and enable pins are declared.
	 */
	int i_m1_in1, i_m1_in2, i_m1_en;
	int i_m2_in1, i_m2_in2, i_m2_en;

	int i_spi_mode, i_spi_speed, i_spi_bits, i_adc_bits;

	/**
	 * Gets parameters from configuration file.
	 */
	n.param<string>("evarobot_driver/driverPath", str_driver_path, "/dev/spidev0.1");
	n.param("evarobot_driver/spiMode", i_spi_mode, 0);
	n.param("evarobot_driver/spiSpeed", i_spi_speed, 1000000);
	n.param("evarobot_driver/spiBits", i_spi_bits, 8);
	n.param("evarobot_driver/adcBits", i_adc_bits, 12);
	n.param<int>("evarobot_driver/M1_IN1", i_m1_in1, 1);
	n.param<int>("evarobot_driver/M1_IN2", i_m1_in2, 12);
	n.param<int>("evarobot_driver/M1_EN", i_m1_en, 5);
	n.param<int>("evarobot_driver/M2_IN1", i_m2_in1, 0);
	n.param<int>("evarobot_driver/M2_IN2", i_m2_in2, 19);
	n.param<int>("evarobot_driver/M2_EN", i_m2_en, 6);
	n.param<double>("evarobot_driver/limitVoltage", d_limit_voltage, 0.63);
	n.param<double>("evarobot_driver/timeout", d_timeout, 0.5);

	if(!n.getParam("evarobot_driver/maxLinearVel", d_max_lin_vel))
	{
			ROS_INFO(GetErrorDescription(-82).c_str());
			i_error_code = -82;
	}

	if(!n.getParam("evarobot_driver/maxAngularVel", d_max_ang_vel))
	{
			ROS_INFO(GetErrorDescription(-83).c_str());
			i_error_code = -83;
	}

	if(!n.getParam("evarobot_driver/wheelSeparation", d_wheel_separation))
	{
			ROS_INFO(GetErrorDescription(-84).c_str());
			i_error_code = -84;
	}

	if(!n.getParam("evarobot_driver/wheelDiameter", d_wheel_diameter))
	{
			ROS_INFO(GetErrorDescription(-85).c_str());
			i_error_code = -85;
	}

	if(!n.getParam("evarobot_driver/pwmFrequency", d_frequency))
	{
			ROS_INFO(GetErrorDescription(-86).c_str());
			i_error_code = -86;
	}

	if(!n.getParam("evarobot_driver/pwmDuty", d_duty))
	{
			ROS_INFO(GetErrorDescription(-87).c_str());
			i_error_code = -87;
	}

	if(!n.getParam("evarobot_driver/pwmCounts", i_counts))
	{
			ROS_INFO(GetErrorDescription(-88).c_str());
			i_error_code = -88;
	}

	if(!n.getParam("evarobot_driver/pwmMode", i_mode))
	{
			ROS_INFO(GetErrorDescription(-91).c_str());
			i_error_code = -91;
	}
  
	if(i_mode != IMPWM::PWMMODE && i_mode != IMPWM::MSMODE)
	{
			ROS_INFO(GetErrorDescription(-89).c_str());
			i_error_code = -89;
	}

	// Dynamic Reconfigure
	dynamic_reconfigure::Server<evarobot_driver::ParamsConfig> srv;
	dynamic_reconfigure::Server<evarobot_driver::ParamsConfig>::CallbackType f;
	f = boost::bind(&CallbackReconfigure, _1, _2);
	srv.setCallback(f);

	/**
	 * SPI mode is set.
	 */
	switch(i_spi_mode)
	{	
		case 0:
		{
			u_c_spi_mode = SPI_MODE_0;
			break;
		}

		case 1:
		{
			u_c_spi_mode = SPI_MODE_1;
			break;
		}

		case 2:
		{
			u_c_spi_mode = SPI_MODE_2;
			break;
		}

		case 3:
		{
			u_c_spi_mode = SPI_MODE_3;
			break;
		}

		default:
		{
			ROS_INFO(GetErrorDescription(-90).c_str());
			i_error_code = -90;
		}
	}
	
	/**
	 * SPI and ADC class references.
	 */
	IMSPI * p_im_spi;
	IMADC * p_im_adc;

	/**
	 * Motor input and enable pins are declared.
	 */
	boost::shared_ptr<IMGPIO> gpio_m1_in[2];
	boost::shared_ptr<IMGPIO> gpio_m2_in[2];
	IMGPIO * gpio_m1_en;
	IMGPIO * gpio_m2_en;

	/**
	 * Creating spi and adc objects.
	 */
	try {
		p_im_spi = new IMSPI(str_driver_path, u_c_spi_mode, i_spi_speed, i_spi_bits);
		p_im_adc = new IMADC(p_im_spi, i_adc_bits);
	} catch(int e) {
		ROS_INFO(GetErrorDescription(e).c_str());
		i_error_code = e;
		return 0;
	}

	try {
		/**
		 * Motor input and enable pins are created from GPIO class.
		 */
		stringstream ss1, ss2;

		ss1 << i_m1_in1;
		ss2 << i_m1_in2;

		gpio_m1_in[0] = boost::shared_ptr< IMGPIO >( new IMGPIO(ss1.str()) );
		gpio_m1_in[1] = boost::shared_ptr< IMGPIO >( new IMGPIO(ss2.str()) );

		ss1.str("");
		ss2.str("");
		ss1 << i_m2_in1;
		ss2 << i_m2_in2;

		gpio_m2_in[0] = boost::shared_ptr< IMGPIO >( new IMGPIO(ss1.str()) );
		gpio_m2_in[1] = boost::shared_ptr< IMGPIO >( new IMGPIO(ss2.str()) );

		ss1.str("");
		ss2.str("");
		ss1 << i_m1_en;
		ss2 << i_m2_en;

		gpio_m1_en = new IMGPIO(ss1.str());
		gpio_m2_en = new IMGPIO(ss2.str());
	} catch(int e) {
		ROS_INFO(GetErrorDescription(e).c_str());
		i_error_code = e;
		return 0;
	}

	/**
	 * evarobot_rgb/SetRGB service client is created to change mode of rgb leds.
	 */
	ros::ServiceClient client = n.serviceClient<im_msgs::SetRGB>("evarobot_rgb/SetRGB");

	/**
	 * imdriver object is created to apply velocities.
	 */
	IMDRIVER imdriver(d_limit_voltage, (float)d_max_lin_vel, (float)d_max_ang_vel,
										(float)d_wheel_separation, (float)d_wheel_diameter,
										d_frequency, i_counts, d_duty, i_mode,
										gpio_m1_in, gpio_m2_in, gpio_m1_en, gpio_m2_en, p_im_adc, client, d_timeout);

	/**
	 * Subscriber for cntr_wheel_vel topic.
	 */
	ros::Subscriber sub = n.subscribe("cntr_wheel_vel", 2, &IMDRIVER::CallbackWheelVel, &imdriver);

	/**
	 * Publisher topic is created with motor_voltages topic name and im_msgs::Voltage message type.
	 */
	ros::Publisher pub = n.advertise<im_msgs::Voltage>("motor_voltages", 1);

	/**
	 * Set diagnostics to handle and publish error.
	 */
	diagnostic_updater::Updater updater;
	updater.setHardwareID("None");
	updater.add("MotorVoltages", &imdriver, &IMDRIVER::ProduceDiagnostics);

	/**
	 * If there is no publisher, writes warning.
	 */
	if(sub.getNumPublishers() < 1)
	{
		ROS_WARN("No appropriate subscriber.");
	}

	/**
	 * Define frequency
	 */
	ros::Rate loop_rate(10.0);

	while(ros::ok())
	{		
		/**
		 * Checks mottor current error or timeout error occured.
		 * Publishes motor voltages.
		 */
		imdriver.CheckMotorCurrent();
		imdriver.CheckTimeout();
		pub.publish(imdriver.GetMotorVoltage());

		/**
		 * Loop is slept to hold frequency.
		 */
		updater.update();
		ros::spinOnce();
		loop_rate.sleep();
	}
  
	return 0;
}
