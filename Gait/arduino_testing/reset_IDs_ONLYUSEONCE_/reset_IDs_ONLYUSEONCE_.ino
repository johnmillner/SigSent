#include <DynamixelSerial3.h>

int OG_IDs[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
int newIDs[] = {15, 16, 17, 12, 13, 14, 9, 10, 11, 6, 7, 8, 3, 4, 5, 0, 1, 2};
int i;

void setup(){

  Dynamixel.begin(1000000,2);  // Initialize the servo at 1Mbps and Pin Control 2
  randomSeed(analogRead(0));
  delay(7000);

//  for(i=0; i<18; i++)
//  {
//    Dynamixel.setID(i, i+18);
//    test_servoID(i+18);
//  }

//delay(10000);

  for(i=0; i<18; i++)
  {
    Dynamixel.setID(OG_IDs[i]+18, newIDs[i]);
    test_servoID(newIDs[i]);
  }
}


void test_servoID(int ID){
//  int new_position = random(412,612);
  Dynamixel.move(ID, 512);
//  while(Dynamixel.moving(ID)){}
}

void loop()
{

}
