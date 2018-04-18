#include <ESC.h>
#include <SPI.h>
#include "Arduino.h"
#include "AX12A.h"
#include <Servo.h>

int torque_level = 1023; //sets torque to ~90% as default

byte mtr1_pin, mtr2_pin, mtr3_pin, mtr4_pin;

ESC mtr1(mtr1_pin, 1148, 1832, 1500), mtr2(mtr2_pin, 1148, 1832, 1500), mtr3(mtr3_pin, 1148, 1832, 1500), mtr4(mtr4_pin, 1148, 1832, 1500);
ESC motors[] = {mtr1, mtr2, mtr3, mtr4};
int walking_gait_tripod[][18] = {
                      {512, 426, 512, 512, 596, 512, 512, 596, 512, 512, 426, 512, 512, 426, 512, 512, 596, 512},
                      {512, 596, 324, 512, 426, 512, 512, 426, 700, 512, 596, 550, 512, 596, 324, 512, 426, 512},
                      {512, 596, 324, 512, 426, 512, 650, 426, 700, 512, 596, 550, 275, 596, 324, 512, 426, 512},
                      {512, 596, 512, 512, 426, 512, 650, 426, 474, 512, 596, 550, 275, 596, 512, 512, 426, 512},
                      {512, 596, 512, 512, 426, 512, 374, 426, 474, 512, 596, 550, 650, 596, 512, 512, 426, 512}
                      };
                      
volatile byte marker;
  
// This flag is true if we are executing a command
// byte doing_command = 0;
volatile byte message_data = -1;
// byte receiving_header = 1;

union
  {
    int servo_state;
    char data[2];
  } message;

volatile int command = -1;


ISR (SPI_STC_vect)
{
      // grab byte from SPI Data Register
      message_data = SPDR;
      Serial.println(message_data);
      
      if (command == -1)
      {
        command = message_data;
      }
      else 
      {
         //put data in butter
        message.data[marker] = message_data;
        marker++;  
      }
     
  }

void setup_torque(int torque)
{
  int id;
  for (id=0; id<18;id++)
  {
   ax12a.setMaxTorque(id, 1023);
  }
}

void setup()
{

    mtr1.arm();
    mtr2.arm();
    mtr3.arm();
    mtr4.arm();
  
    // have to send on master in, *slave out*
    pinMode(MISO, OUTPUT);

    // turn on SPI in slave mode and attach the interrupt
    SPCR |= _BV(SPE);
    SPCR |= _BV(SPIE);

    SPI.setDataMode(SPI_MODE0);
    
    marker = 0;
    
    ax12a.begin(1000000, 2, &Serial3);
    Serial.begin(115200);
    command = -1;
    setup_torque(torque_level);
}

void loop()
{
//    Serial.print("marker:");
//    Serial.println(marker);
    while(marker < 2)
    {
      if (marker == 2)
        break;
//      Serial.println(marker);
      //do nothing until buffer is fixed
    }
    Serial.println(command);
//    Serial.println("I'm here.");
    int position = message.servo_state & 1023;
    int id = (message.servo_state >> 10) & 63;
    
    if (command == 20)
    {
      Serial.print("Id: ");
      Serial.print(id);
      Serial.print("  Position: ");
      Serial.print(position);
      
      
      ax12a.move(id, position);  
    }

    if (command == 30)
    {
      Serial.print("Setting ID: ");
      Serial.print(id);
      Serial.print(" to new ID ");
      Serial.println(position);
      // Assign new ID, ignore the shitty naming conventions
      ax12a.setID(id, position);

      // Test moving to a random val to see if assigning worked
      ax12a.move(position, random(400, 601));
    }

   if (command == 40)
   {
      //motors[id].speed(position);
   }
    
    command = -1;
    marker = 0;
}

