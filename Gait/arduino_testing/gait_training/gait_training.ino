#include <SPI.h>

ServoState ss;
int leg_index;
int gait[12][6][3];
volatile bool process_it;
char step = 0;
char leg = 0;
char servo = 0;

union ServoState
{
  long leg_state;
  byte state_array[4];
};

ISR (SPI_STC_vect)
{
   // grab byte from SPI Data Register
   byte data = SPDR;
   
   char num_servos_set = (step * servo * leg + leg * servo + servo);

   // Add to gait if we haven't set everything
   if (num_servos_set < sizeof gait)
   {
       gait[step][leg][servo] = data;

       servo++;

       if (servo > 2)
       {
           servo = 0;
           leg++;

           if (leg > 5)
           {
               step++;
           }
       }
   }
}

int get_servo_state(long servo_state, byte servo_num)
{
  long servo_mask = 1023;
  long individual_servo;
  
  for (byte i = 0; i < servo_num; i++)
  {
    servo_mask = servo_mask << 10;
  }
  Serial.println(servo_state, BIN);
  Serial.println(servo_mask, BIN);
  individual_servo = servo_state & servo_mask;
  
  for (byte i = 0; i < servo_num; i++)
  {
    individual_servo = individual_servo >> 10;
  }

  Serial.println(individual_servo, BIN);
  
  return (int)individual_servo;
}

byte get_command(long servo_state)
{
  Serial.println(servo_state, BIN);
  int command = (servo_state >> 30) & 3;
  Serial.println(command, BIN);

  return (byte)command;
}

void setup() {
// have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // now turn on interrupts
  SPI.attachInterrupt();
  
  leg_index = 0;
  process_it = false;

    Serial.begin(9600);      // open the serial port at 9600 bps:    

}

void loop() {
  
  if (process_it)
  {
    Serial.print("Command: ");
    Serial.println(get_command(ss.leg_state));

    for (byte i = 0; i < 3; i++)
    {
      Serial.print("Servo: " + i);
      Serial.println(get_servo_state(ss.leg_state, i));
      Serial.println();
    }
    
    process_it = false;
  }
}