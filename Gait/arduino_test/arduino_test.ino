/**
 * SigSent MCU code
 *
 * written by Richie Wales and Joshua Lee Franco
 *
 * This program runs the base protocols that operates sigsent's control over its
 * legs/joints
 *
 *
 *Walking mode angles
 *0 512 1 426 2 512 3 512 4 596 5 512 6 512 7 596 8 512 9 512 10 426 11 512 12 512 13 426 14 512 15 512 16 596 17 512
 *512 426 512 512 596 512 512 596 512 512 426 512 512 426 512 512 596 512
 *
 *Drive mode angles
 *0 358 1 426 2 512 3 664 4 596 5 512 6 512 7 700 8 200 9 512 10 323 11 825 12 664 13 426 14 512 15 358 16 596 17 512
 *358 426 512 664 596 512 512 700 200 512 323 825 664 426 512 358 596 512
 *
 *SigSent joint numbering scheme
 * 5                  2
 *   4              1
 *     3__________0
 *     |          |
 *     |          |
 * 8 7 6          9 10 11
 *     |          |
 *     |          |
 *    15__________12
 *  16              13
 * 17                  14
 *
 *
 */


#include <SPI.h>
#include <DynamixelSeria3.h>

// These definitions came from the pi_spi_modules.py file that define the message
// class and its generation functions :)
byte mode_change_header = 0b00110000;
byte walking_move_header = 0b00001100;
byte esc_header = 0b00000011;
byte fwd = 0b00000000;
byte left = 0b00001111;
byte right = 0b11110000;
byte back = 0b11111111;
byte driving_mode = 0b00001111;
byte walking_mode = 0b11110000;
// These definitions are the error bits we are worried about definited in
// AX-12A documentation
byte overload_error = 0b00100000;
byte range_error = 0b00001000;
byte overheating_error = 0b00000100;
byte angle_limit_error = 0b00000010;
// These are angles defined by us to constitute as the default stances for
// sigsent to move to for each mode and incase any error in incountered
int default_walking[] = {512, 426, 512, 512, 596, 512, 512, 596, 512, 512, 426, 512, 512, 426, 512, 512, 596, 512};
int default_driving[] = {358, 426, 512, 664, 596, 512, 512, 700, 200, 512, 323, 825, 664, 426, 512, 358, 596, 512};

int walking_gait_tripod[][] = {{512, 426, 512, 512, 596, 512, 512, 596, 512, 512, 426, 512, 512, 426, 512, 512, 596, 512},
                      {},
                      {}}
// Lets the MCU to know what to expect for the next message to parse
// it correctly
byte receiving_header = 1;
byte receiving_mode = 0;
byte receiving_walking_move = 0;
byte receiving_esc_direction = 0;
byte receiving_esc_speed = 0;

// General Variables for usage in the program
byte return_status_byte = 0; //used when returning from a function for status of its execution
int id = 0; //general id variable for storing servo IDs
int counter= 0; //general counter
float diff = 0;
float tolerance = 0.1;
float current_tolerance = 0;



enum ModeValue
{
    WALKING_MODE,
    DRIVING_MODE,
    MODE_ERROR
};

enum Direction
{
    FWD,
    LEFT,
    RIGHT,
    BACK,
    DIR_ERROR
};

enum MessageType
{
    MODE_TYPE,
    WALKING_TYPE,
    ESC_TYPE,
    MESSAGE_ERROR
};

Direction esc_dir;

ModeValue get_mode(byte mode)
{
    if (mode == walking_mode)
        return WALKING_MODE;
    if (mode == driving_mode)
        return DRIVING_MODE;

    return MODE_ERROR;
}

MessageType get_message_type(byte header)
{
    if (header == mode_change_header)
        return MODE_TYPE;
    if (header == walking_move_header)
        return WALKING_TYPE;
    if (header == esc_header)
        return ESC_TYPE;

    return MESSAGE_ERROR;
}

Direction get_direction(byte dir)
{
    if (dir == fwd)
        return FWD;
    if (dir == left)
        return LEFT;
    if (dir == right)
        return RIGHT;
    if (dir == back)
        return BACK;

    return DIR_ERROR;
}

//#####################################################
//# Josh, the three functions below are ALL YOU BABY  #
//# switch_mode, walk_move, drive_escs                #
//### ##################################################
byte switch_mode(ModeValue mode)
{
    // Depending on the mode, do some stuff to the servos

    // Return 1 for success, 0 for failure
    return 1;
}

byte walk_move(Direction dir)
{

    return 1;
}

// We don't have to have a parameter since the direction value is
// a global value that was set in the previous SPI interrupt, but I
// think its more readable to know exactly is being passed to it when tracing the caller
byte drive_escs(Direction dir, int speed)
{

    return 1;
}

ISR (SPI_STC_vect)
{
    // grab byte from SPI Data Register
    byte data = SPDR;

    //Serial.print("Received: ");
    Serial.println(data);

    if (receiving_header)
    {
        //Serial.println("Receiving header");
        MessageType message_type = get_message_type(data);

        if (message_type == MESSAGE_ERROR)
            return;

        receiving_header = 0;

        if (message_type == MODE_TYPE)
            receiving_mode = 1;
        else if (message_type == WALKING_TYPE)
            receiving_walking_move = 1;
        else if (message_type == ESC_TYPE)
            receiving_esc_direction = 1;
    }

    else if (receiving_mode)
    {
        ModeValue mode = get_mode(data);

        if (mode == MODE_ERROR)
        {
            // Failure, start over
            receiving_header = 1;
            receiving_mode = 0;

            return;
        }

        // Got the mode, now do the crazy servo code that switches mode
        byte ret = switch_mode(mode);

        if (ret == 0)
        {
            ;// Failed to switch mode, how do we handle this?
        }

        receiving_mode = 0;
        receiving_header = 1;
    }

    else if (receiving_walking_move || receiving_esc_direction)
    {
        //Serial.println("Receiving direction");
        Direction dir = get_direction(data);

        if (dir == DIR_ERROR)
        {
            if (receiving_walking_move)
                receiving_walking_move = 0;
            else
                receiving_esc_di  rection = 0;

            receiving_header = 1;
        }

        if (receiving_walking_move)
        {
            byte ret = walk_move(dir);

            if (ret == 0)
            {
                ;
            }

            receiving_header = 1;
            receiving_walking_move = 0;
        }

        else if (receiving_esc_direction)
        {
            esc_dir = dir;
            receiving_esc_direction = 0;
            receiving_esc_speed = 1;
        }
    }

    else if (receiving_esc_speed)
    {
        //Serial.println("Receiving speed");
        // speed is just a plain old 8 bit number, no processing needed, just do it
        byte ret = drive_escs(esc_dir, data);

        if (ret == 0)
        {
            ;
        }

        receiving_header = 1;
        receiving_esc_speed = 0;
        esc_dir = DIR_ERROR;
    }

}

// This function checks all servos if there are any errors for overloading or
// over torque
// return 1 if one of the servos is in overloading
// return 0 if none of the servos are in overloading
//
// further improve by giving back servos that are in overload to just reset
// those servos and move them back into place
byte check_servos_for_overload()
{
  for(id=17;id>=0;id--)
  {
    return_status_byte=Dynamixel.ping(id);

    if (return_status_byte & overload_error)
    {
      return 1;
    }
  }
  return 0;
}

// This function checks if all the servos have achieved their position with a
// -+10% tolerance and if not corrects the servo takes in current
byte check_servo_positions(int current_gait_position)
{
    // Need to check if servos are in position and if not then correct
    for(id=17;id>=0;id--)
    {
      return_status_byte = ten_percent_tolerance_check(id, walking_gait_tripod[current_gait_position][id]);

      if (return_status_byte == 1)
      {
        continue;
      }
      else
      {
        while(return_status_byte != 1)
        {
          Dynamixel.move(ID, walking_gait_tripod[current_gait_position][id]);
          delay(200);//
          return_status_byte = ten_percent_tolerance_check(id, walking_gait_tripod[current_gait_position][id]);
        }
      }
    }
}

// This fucntion specifically checks if a servo position is within 10% tolerance
// of its commanded/assumed position.
byte ten_percent_tolerance_check(int id, int position_to_be_checked)
{

    diff = (float)abs(position_to_be_checked - Dynamixel.readPosition(id));
    current_tolerance = (float)position_to_be_checked*0.10;

    if (diff < current_tolerance)
    {
      //position is within 10% tolerance
      return 1;
    }
    else
    {
      //position is out of 10% tolerance
      return 0;
    }
}

void setup()
{
    // have to send on master in, *slave out*
    pinMode(MISO, OUTPUT);

    // turn on SPI in slave mode and attach the interrupt
    SPCR |= _BV(SPE);
    SPI.attachInterrupt();
    Dynamixel.begin(1000000,2);  // Initialize the servo at 1Mbps and Pin Control 2
    delay(1000);


    //Serial.begin(9600);      // open the serial port at 9600 bps:

}

void loop()
{

}
