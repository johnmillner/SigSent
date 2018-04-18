#include <DynamixelSerial3.h>

#include <ESC.h>
#include <SPI.h>
//#include "Arduino.h"
//#include "AX12A.h"

#include <Servo.h>

int torque_level = 1023; //sets torque to ~90% as default
int speed = 200;
int turning_speed = 150;

volatile byte command = 0;

int walking_zero_two_four[][18] = {
//                     0     1    2    3    4    5    6    7 .  8 .  9 .  10 . 11 . 12 . 13 . 14 . 15 . 16 . 17
                      {512, 596, 512, 512, 426, 512, 512, 426, 474, 512, 596, 550, 512, 596, 512, 512, 426, 512},
                      {512, 596, 324, 512, 426, 512, 600, 426, 700, 512, 596, 550, 400, 596, 324, 512, 426, 512},
                      {512, 596, 512, 512, 426, 512, 600, 426, 474, 512, 596, 550, 400, 596, 512, 512, 426, 512},
                      {624, 596, 512, 512, 426, 512, 425, 426, 474, 512, 596, 550, 512, 596, 512, 512, 426, 512},
                      {624, 596, 324, 512, 426, 512, 425, 426, 700, 512, 596, 550, 512, 596, 324, 512, 426, 512}
                      };

int walking_one_three_five[][18] = {
                      {512, 596, 512, 512, 426, 512, 512, 426, 474, 512, 596, 550, 512, 596, 512, 512, 426, 512},
                      {512, 596, 512, 512, 426, 700, 512, 426, 474, 425, 596, 324, 512, 596, 512, 624, 426, 700},
                      {512, 596, 512, 512, 426, 512, 512, 426, 474, 425, 596, 550, 512, 596, 512, 624, 426, 512},
                      {512, 596, 512, 400, 426, 512, 512, 426, 474, 600, 596, 550, 512, 596, 512, 512, 426, 512},
                      {512, 596, 512, 400, 426, 700, 512, 426, 474, 600, 596, 324, 512, 596, 512, 512, 426, 700}
                      };

int walking_gait_tripod[][18] = {
                      {512, 596, 512, 512, 426, 512, 512, 426, 474, 512, 596, 550, 512, 596, 512, 512, 426, 512},
                      {600, 596, 512, 500, 426, 700, 400, 426, 474, 400, 596, 324, 500, 596, 512, 600, 426, 700},
                      {425, 596, 324, 500, 426, 512, 625, 426, 700, 400, 596, 550, 275, 596, 324, 600, 426, 512},
                      {425, 596, 512, 275, 426, 512, 625, 426, 474, 625, 596, 550, 500, 596, 512, 524, 426, 512}
};
int default_driving[] = {358, 596, 512, 664, 426, 512, 512, 323, 825, 512, 700, 200, 664, 596, 512, 358, 426, 512};

// This is triangle
//int zero_two_four_steps[5] =  {0, 1, 2, 3, 3};
//int one_three_five_steps[5] = {0, 3, 3, 1, 2};

// This is square
int zero_two_four_steps[10] =  {0, 1, 2, 2, 3, 3, 4};
int one_three_five_steps[10] = {0, 3, 3, 4, 1, 2, 2};

int zero_two_four_turn_steps[10] =  {0, 1, 2, 0, 0};
int one_three_five_turn_steps[10] = {0, 0, 0, 1, 2};

void move_fwd(int zero_two_four_step, int one_three_five_step)
{
    for (int i = 0; i < 18; i++)
    {
      int leg = i / 3;

      // 0-2-4
      if (leg % 2 == 0)
      {
        Dynamixel.moveSpeed(i, walking_zero_two_four[zero_two_four_step][i], speed);
      }
      else
      {
        Dynamixel.moveSpeed(i, walking_one_three_five[one_three_five_step][i], speed);
      }
    }
}

void turn_right()
{
  for (int i = 0; i < 5; i++)
  {
    for (int j = 0; j < 18; j++)
    {
      int leg = i / 3;

      // 0-2-4
      if (leg % 2 == 0)
      {
        Dynamixel.moveSpeed(i, walking_zero_two_four[zero_two_four_turn_steps[i]][j], speed);
      }
      else
      {
        Dynamixel.moveSpeed(i, walking_one_three_five[one_three_five_turn_steps[i]][j], speed);
      } 
    }
  }

  delay(600);
}

ISR (SPI_STC_vect)
{
    // grab byte from SPI Data Register
    command = SPDR;
    
    Serial.println(command);
}

void enter_driving()
{
  for (int i = 0; i < 18; i++)
  {
    Dynamixel.moveSpeed(i, default_driving[i], speed);
  }

  delay(600);
}

void enter_walking()
{
  for (int i = 0; i < 18; i++)
  {
    Dynamixel.moveSpeed(i, walking_zero_two_four[0][i], speed);
  }

  delay(600);
}

void walk()
{
  for (int i = 1; i < 7; i++)
  {
    move_fwd(zero_two_four_steps[i], one_three_five_steps[i]);
    delay(600);
  }
}
                      
void setup_torque(int torque)
{
  int id;
  for (id=0; id<18;id++)
  {
   Dynamixel.setMaxTorque(id, 1023);
   Dynamixel.setPunch(id, 0);
  }
}

void setup()
{
    Serial.begin(115200);
    pinMode(MISO, OUTPUT);

    // turn on SPI in slave mode and attach the interrupt
    SPCR |= _BV(SPE);
    SPCR |= _BV(SPIE);

    SPI.setDataMode(SPI_MODE0);
    Dynamixel.begin(1000000, 2);
    setup_torque(torque_level);

    for (int j = 0; j < 18; j++)
    {
      Dynamixel.moveSpeed(j, walking_gait_tripod[0][j], speed);
    }

      delay(5000);

  
     
}

void loop()
{
  if (command == 10)
  {
    walk();
    command = 0;                                                              
  }
  else if (command == 20)
  {
    enter_driving();
    command = 0;
  }

  else if (command == 30)
  {
    enter_walking();
    command = 0;
  }

  else if (command == 40)
  {
    turn_right();
    command = 0;
  }
  
}

