

#include <SPI.h>
#include <DynamixelSerial3.h>

volatile byte marker;
  
// This flag is true if we are executing a command
// byte doing_command = 0;
byte message_data = -1;
// byte receiving_header = 1;

union
  {
    int servo_state;
    char data[2];
  } message;


ISR (SPI_STC_vect)
{
      // grab byte from SPI Data Register
      message_data = SPDR;
      //Serial.println(message_data);
      

      //put data in butter
      message.data[marker] = message_data;
      marker++;
      //Serial.print("marker:");
      //Serial.println(marker);

      //Serial.print("Received: ");

      // 10101010 will be an alert pi sends to MCU when it has the OK
      // to send a command. Tells MCU to expect a message header
      // if (message_data == 0b10101010)
      // {
      //     doing_command = 1;
      //     receiving_header = 1;
      // }
  }

void setup()
{
    // have to send on master in, *slave out*
    pinMode(MISO, OUTPUT);

    // turn on SPI in slave mode and attach the interrupt
    SPCR |= _BV(SPE);
    SPCR |= _BV(SPIE);

    SPI.setDataMode(SPI_MODE0);
    marker = 0;
    Dynamixel.begin(1000000,2);
    Serial.begin(115200);
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

//    Serial.println("I'm here.");

    int position = message.servo_state & 1023;
    int id = (message.servo_state >> 10) & 63;
    Serial.print("Id: ");
    Serial.print(id);
    Serial.print("  Position: ");
    Serial.print(position);
    
    
    Dynamixel.move(id, position);
    marker = 0;
}

