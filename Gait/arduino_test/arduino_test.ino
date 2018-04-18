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
 *0 512 1 596 2 512 3 512 4 426 5 512 6 512 7 426 8 512 9 512 10 596 11 512 12 512 13 596 14 512 15 512 16 426 17 512
 *512 596 512 512 426 512 512 426 474 512 596 550 512 596 512 512 426 512
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
#include <DynamixelSerial3.h>

int torque_level = 1023; //sets torque to ~90% as default


// This flag is true if we are executing a command
byte doing_command = 0;
byte message_data[10] = {0};
byte prev_msg = -1;
byte next_msg = 0;

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
int default_driving[] = {358, 596, 512, 664, 426, 512, 512, 323, 825, 512, 700, 200, 664, 596, 512, 358, 426, 512};

int speed = 200;
int gait_length = 5;
int walking_zero_two_four[][18] = {
//                     0     1    2    3    4    5    6    7 .  8 .  9 .  10 . 11 . 12 . 13 . 14 . 15 . 16 . 17
                      {512, 596, 512, 512, 426, 512, 512, 426, 474, 512, 596, 550, 512, 596, 512, 512, 426, 512},
                      {512, 596, 324, 512, 426, 512, 600, 426, 700, 512, 596, 550, 400, 596, 324, 512, 426, 512},
                      {512, 596, 512, 512, 426, 512, 600, 426, 474, 512, 596, 550, 400, 596, 512, 512, 426, 512},
                      {624, 596, 512, 512, 426, 512, 425, 426, 474, 512, 596, 550, 512, 596, 512, 512, 426, 512},
                      {624, 596, 324, 512, 426, 512, 425, 426, 700, 512, 596, 550, 512, 596, 324, 512, 426, 512}
                      };

int walking_one_three_five[][18] = {
//                     0     1    2    3    4    5    6    7 .  8 .  9 .  10 . 11 . 12 . 13 . 14 . 15 . 16 . 17
                      {512, 596, 512, 512, 426, 512, 512, 426, 474, 512, 596, 550, 512, 596, 512, 512, 426, 512},
                      {512, 596, 512, 512, 426, 700, 512, 426, 474, 425, 596, 324, 512, 596, 512, 624, 426, 700},
                      {512, 596, 512, 512, 426, 512, 512, 426, 474, 425, 596, 550, 512, 596, 512, 624, 426, 512},
                      {512, 596, 512, 400, 426, 512, 512, 426, 474, 600, 596, 550, 512, 596, 512, 512, 426, 512},
                      {512, 596, 512, 400, 426, 700, 512, 426, 474, 600, 596, 324, 512, 596, 512, 512, 426, 700}
                      };

int one_three_five_steps[5] = {0, 3, 3, 1, 2};
int zero_two_four_steps[5] =  {0, 1, 2, 3, 3};

void move_fwd(int zero_two_four_step, int one_three_five_step)
{
    for (int i = 0; i < 18; i++)
    {
      int leg = i / 3;

      // 0-2-4
      if (leg % 2 == 0)
      {
        Dynamixel.moveSpeed(i, walking_zero_two_four[zero_two_four_steps[zero_two_four_step]][i], speed);
      }
      else
      {
        Dynamixel.moveSpeed(i, walking_one_three_five[one_three_five_steps[one_three_five_step]][i], speed);
      }
    }
}
int gait_state = 0;

// Lets the MCU to know what to expect for the next message to parse
// it correctly
byte receiving_header = 1;
byte receiving_mode = 0;
byte receiving_walking_move = 0;
byte receiving_esc_direction = 0;
byte receiving_esc_speed = 0;

//flags
byte ready_flag = 0;

// General Variables for usage in the program
byte return_status_byte = 0; //used when returning from a function for status of its execution
int id = 0; //general id variable for storing servo IDs
int counter= 0; //general counter
float diff = 0;
float tolerance = 0.1;
float current_tolerance = 0;
//int number_of_gait_states = sizeof (walking_gait_tripod) / sizeof (walking_gait_tripod[0]);

enum ModeValue
{
    WALKING_MODE,
    DRIVING_MODE,
    MODE_ERROR
};

enum Direction
{
    FWD,
    LFT,
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

int get_mode(byte mode)
{
    if (mode == walking_mode)
        return WALKING_MODE;
    if (mode == driving_mode)
        return DRIVING_MODE;

    return MODE_ERROR;
}

int get_message_type(byte header)
{
    Serial.println(header);
    Serial.println(walking_move_header);
    if (header == mode_change_header)
        return MODE_TYPE;
    if (header == walking_move_header)
        return WALKING_TYPE;
    if (header == esc_header)
        return ESC_TYPE;

    return MESSAGE_ERROR;
}

int get_direction(byte dir)
{
    if (dir == fwd)
        return FWD;
    if (dir == left)
        return LFT;
    if (dir == right)
        return RIGHT;
    if (dir == back)
        return BACK;

    return DIR_ERROR;
}

byte switch_mode(int mode)
{
    // Depending on the mode, do some stuff to the servos
    if (mode == WALKING_MODE)
    {
        gait_state = 0;
        walk_move(FWD);
    }
    else if (mode == DRIVING_MODE)
    {
      default_driving_stance();
    }

    // Return 1 for success, 0 for failure
    return 1;
}

byte walk_move(int dir)
{
    digitalWrite(42, HIGH);  
    delay(1000);               
    digitalWrite(42, LOW);    
    delay(1000);
    // Commented out until we actually have a full gait to move through
     move_fwd(zero_two_four_steps[gait_state], one_three_five_steps[gait_state]);
    // check_servo_positions(-1);

  gait_state++;

  if (gait_state > gait_length)
  {
    // Skip the default orientation in walking mode
    gait_state = 1;
  }
  
  delay(600);
  
  digitalWrite(42, HIGH);  
  delay(100);               
  digitalWrite(42, LOW);    
  delay(100);
  
  return 1;
}

// This function puts SigSent into the default walking stance
byte default_walking_stance()
{
  for (id=0; id<18;id++)
  {
  
    Dynamixel.moveSpeed(id, walking_zero_two_four[id], speed);

    //check_servo_positions(-1);
  }
}

byte default_driving_stance()
{
  for (id=0; id<18;id++)
  {
    Dynamixel.moveSpeed(id, default_driving[id], speed);
  }

  delay(600);
}

// We don't have to have a parameter since the direction value is
// a global value that was set in the previous SPI interrupt, but I
// think its more readable to know exactly is being passed to it when tracing the caller
byte drive_escs(int dir, int speed)
{

    return 1;
}

ISR (SPI_STC_vect)
{
    // grab byte from SPI Data Register
    byte temp = SPDR;
    
    message_data[next_msg++] = temp;

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
  for(id=0; id<18;id++)
  {
    return_status_byte = (byte)Dynamixel.ping(id);

    if (return_status_byte & overload_error)
    {
      // This means ther servos are in overload
      return 1;
    }
  }
  // This means the servos aren't in overload
  return 0;
}

byte check_servos_for_overheat()
{
  for(id=0; id<18;id++)
  {
    return_status_byte = (byte)Dynamixel.ping(id);

    if (return_status_byte & overheating_error)
    {
      // This means ther servos are in overheat
      return 1;
    }
  }
  // This means the servos aren't in overheat
  return 0;
}

// This function checks if all the servos have achieved their position with a
// -+10% tolerance and if not corrects the servo takes in current
//byte check_servo_positions(int current_gait_position)
//{
//  if (current_gait_position >  (number_of_gait_states-1))
//  {
//    //
//    return 0;
//  }
//
//  for(id=0; id<18;id++)
//  {
//    if (current_gait_position == -1)
//    {
//      return_status_byte = ten_percent_tolerance_check(id, walking_gait_tripod[current_gait_position][id]);
//    }
//    else
//    {
//      return_status_byte = ten_percent_tolerance_check(id, walking_gait_tripod[current_gait_position][id]);
//
//    }
//
//    if (return_status_byte == 1)
//    {
//      continue;
//    }
//    else
//    {
//      while(return_status_byte != 1)
//      {
//        Dynamixel.move(id, walking_gait_tripod[current_gait_position][id]);
//        delay(200);//
//        return_status_byte = ten_percent_tolerance_check(id, walking_gait_tripod[current_gait_position][id]);
//      }
//    }
//  }
//}

// This fucntion specifically checks if a servo position is within 10% tolerance
// of its commanded/assumed position.
byte ten_percent_tolerance_check(int id, int position_to_be_checked)
{
  // Get the difference of the assumed and the current position to find the tolerance
  diff = (float)abs(position_to_be_checked - Dynamixel.readPosition(id));
  // create the value of tolerance based of the current assumed servo position.
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

void setup_torque(int torque)
{
  for (id=0; id<18;id++)
  {
  Dynamixel.setMaxTorque(id,torque);
  }
}

void reset_messages()
{
    next_msg = 0;
    for (int i = 0; i < 10; i++)
      message_data[i] = -1;
}

void setup() 
{
  receiving_header = 1;
  reset_messages();
    pinMode(42, OUTPUT);
    Serial.begin(9600);
    // have to send on master in, *slave out*
    pinMode(MISO, OUTPUT);

    // turn on SPI in slave mode and attach the interrupt
    SPCR |= _BV(SPE);
    SPCR |= _BV(SPIE);

    SPI.setDataMode(SPI_MODE0);

    Dynamixel.begin(1000000, 2);
    setup_torque(torque_level);

}

void loop() 
{
    while (message_data[0] == -1)
    {
        Serial.println("Doing nothing");
        // Do nothing
        ready_flag = 0xFF;
        digitalWrite(42, HIGH);  
        delay(500);               
        digitalWrite(42, LOW);    
        delay(500);                       
    }

    if (receiving_header)
    {
        Serial.println("Receiving header");
                
        
        MessageType message_type = get_message_type(message_data[0]);

        if (message_type == MESSAGE_ERROR)
        {
            // ABORT
            receiving_header = 0;
            doing_command = 0;
            reset_messages();
        }
        else
        {
            receiving_header = 0;

            if (message_type == MODE_TYPE)
                receiving_mode = 1;
            else if (message_type == WALKING_TYPE)
                receiving_walking_move = 1;
            else if (message_type == ESC_TYPE)
                receiving_esc_direction = 1;
            
            message_data[0] = -1;
        }
    }

    else if (receiving_mode)
    {
        Serial.println("Receiving mode");
        // This shouldnt be necessary, but just in case...
        while (message_data[1] == -1)
        {
            // Wait for message from PI
        }

        ModeValue mode = get_mode(message_data[1]);

        if (mode == MODE_ERROR)
        {
            // Anything special here?
        }
        else
        {
            // Got the mode, now do the crazy servo code that switches mode
            byte ret = switch_mode(mode);

            if (ret == 0)
            {
                ;// Failed to switch mode, how do we handle this?
            }
        }

        // Whether we fail or succeed, no more to be done, go back into idle mode
        receiving_mode = 0;
        doing_command = 0;
        reset_messages();
    }

    else if ((receiving_walking_move || receiving_esc_direction))
    {
        
        
        // This shouldnt be necessary, but just in case...
        while (message_data[1] == -1)
        {
            // Wait for message from PI
        }

        //Serial.println("Receiving direction");
        Direction dir = get_direction(message_data[1]);

        if (dir == DIR_ERROR)
        {
            if (receiving_walking_move)
                receiving_walking_move = 0;
            else
                receiving_esc_direction = 0;

            doing_command = 0;
            reset_messages();
        }

        else if (receiving_walking_move)
        {
          Serial.println("Receiving walking");
            byte ret = walk_move(dir);

            if (ret == 0)
            {
                ;
            }
            reset_messages();
            doing_command = 0;
            receiving_walking_move = 0;
        }

        else if (receiving_esc_direction)
        {
            reset_messages();
            esc_dir = dir;
            receiving_esc_direction = 0;
            receiving_esc_speed = 1;
        }
    }

    else if (receiving_esc_speed)
    {
        // This shouldnt be necessary, but just in case...
        while (message_data[2] == -1)
        {
            // Wait for message from PI
        }

        //Serial.println("Receiving speed");
        // speed is just a plain old 8 bit number, no processing needed, just do it
        byte ret = drive_escs(esc_dir, message_data[2]);

        if (ret == 0)
        {
            ;
        }
        reset_messages();
        doing_command = 0;
        receiving_esc_speed = 0;

        // Reset this to a neutral value
        esc_dir = DIR_ERROR;
    }
}
