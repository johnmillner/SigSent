#include <ESC.h>
#include <SPI.h>
#include <DynamixelSerial3.h>
#include <Servo.h>

byte mtr1_pin, mtr2_pin, mtr3_pin, mtr4_pin;

ESC mtr1(), mtr2, mtr3, mtr4;
ESC motors[] = {mtr1, mtr2, mtr3, mtr4};

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

void setup()
{

    mtr1.attach(10, 1200, 1800);
    mtr2.attach(11, 1200, 1800);
    mtr3.attach(12, 1200, 1800);
    mtr4.attach(13, 1200, 1800);
  
    // have to send on master in, *slave out*
    pinMode(MISO, OUTPUT);

    // turn on SPI in slave mode and attach the interrupt
    SPCR |= _BV(SPE);
    SPCR |= _BV(SPIE);

    SPI.setDataMode(SPI_MODE0);
    marker = 0;
    Dynamixel.begin(1000000,2);
    Serial.begin(115200);
    command = -1;
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
      
      
      Dynamixel.move(id, position);  
    }

    if (command == 30)
    {
      Serial.print("Setting ID: ");
      Serial.print(id);
      Serial.print(" to new ID ");
      Serial.println(position);
      // Assign new ID, ignore the shitty naming conventions
      Dynamixel.setID(id, position);

      // Test moving to a random val to see if assigning worked
      Dynamixel.move(position, random(400, 601));   
    }

   if (command == 40)
   {
      motors[id].write(position);
   }
    
    command = -1;
    marker = 0;
}

