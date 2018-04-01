//Imported ibraries
#include <DynamixelSerial1.h>

//Global variables
int temperature,voltage, position; //used when status of the servo is requested
int ID =1, newID; //ID refers to current ID or old ID when setting a new ID for servos
int input_position=500, current_position, new_position; //input used as a "buffer" when
int menu_start = 0, menu_printed = 0;
char menu_mode;
int c=0;


//Function prototypes
void print_menu(int sel);
int check_value(int value_selected, int input_value);
void test_servoID(int ID);



void setup(){
Serial.begin(9600);              // Begin Serial Comunication for serial monitor
Dynamixel.begin(1000000,2);  // Initialize the servo at 1Mbps and Pin Control 2
randomSeed(analogRead(0));
delay(1000);

// wait for serial port to connect. Needed for native USB port only
while (!Serial){}
}


void loop(){
  //Start of menu check if the menu has been printed already
  if (menu_start == 0 && menu_printed == 0) {
    print_menu(1);
    menu_printed = 1;
  }
  //otherwise print the escape message to the user for reminder
  else if (menu_start == 1)
    print_menu(2);
//  Serial.println(menu_start);
//  Serial.println(menu_printed);

  if (Serial.available()>0) {
    menu_mode = Serial.read();
    Serial.println(menu_mode);
    if(sizeof(menu_mode) == sizeof(char)){
      switch (menu_mode) {
        case 'm':    //Single moving mode
          Serial.println();
          Serial.println();
          Serial.println("_____Moving mode_____");
          Serial.println("Enter in the form of 'ID Position'. Can string together multiple commands.");
          Serial.println("(Do not use the value of 0 for ID)");
          while(ID > -1){
            if (Serial.available()>0) {
              ID = Serial.parseInt();
              input_position = Serial.parseInt();
//              Serial.println(input);
//                Serial.println(c);
//                c++;
              if (ID > 0 && input_position > 0){
                Serial.print("ID = ");
                Serial.print(ID);
                Serial.print(" ");
                Serial.print("Input position = ");
                Serial.println(input_position);
                Dynamixel.move(ID, input_position);
                while(Dynamixel.moving(ID)){}
              }
            }
          }
          break;
        case '#':    // Mode to set ID numbers
          Serial.println("_____Set ID mode_____");
          Serial.println("Enter in form of 'old_ID new_ID'.");
          Serial.println("(Do not use the value of 0 for ID");
          while(ID > -1){
            if (Serial.available()>0) {
              ID = Serial.parseInt();
              newID = Serial.parseInt();
              if (ID > 0 && newID > 0){
                Serial.print("Old ID = ");
                Serial.print(ID);
                Serial.print(" New ID = ");
                Serial.println(newID);
                Dynamixel.setID(ID, newID);

                Serial.print("Servo ID changed from '");
                Serial.print(ID);
                Serial.print("' to '");
                Serial.print(newID);
                Serial.println("'.");
                Serial.println();
                test_servoID(newID);
              }
            }
          }
          break;
        case 'T':    // Torque mode
          Serial.println("_____Torque limits mode_____");
          Serial.println("Not Ready Yet");
          Dynamixel.setMaxTorque(1,512);
          break;
        case 'a':    // Angle mode
          Serial.println("_____Angle limits mode_____");
          Serial.println("Not Ready Yet");
          break;
        case 'c':    //
          Serial.println("_____Current limits mode_____");
          Serial.println("Not Ready Yet");
          break;
        case 'v':    //
          Serial.println("_____Voltage limits mode_____");
          Serial.println("Not Ready Yet");
//          Dynamixel.setVoltageLimit(1,65,120);  // Set Operating Voltage from 6.5v to 12v
          break;
        case 't':    // your hand is nowhere near the sensor
          Serial.println("_____Temperature limits mode_____");
          Serial.println("Not Ready Yet");
//          Dynamixel.setTempLimit(1,80);  // Set Max Temperature to 80 Celcius
          break;
        default:
          Serial.println("Please give a valid menu input");
      }
      //reset menu flags & ID(used as a flag as well)
          menu_printed = menu_start = 253;
          ID = 1;
          Serial.println();
          Serial.println();
          Serial.println();
          Serial.println();
          
    }

//    temperature = Dynamixel.readTemperature(input_position); // Request and Print the Temperature
//    voltage = Dynamixel.readVoltage(input_position);         // Request and Print the Voltage
//    position = Dynamixel.readPosition(input_position);       // Request and Print the Position
    //Dynamixel.move(1,random(200,800));  // Move the Servo radomly from 200 to 800
//    Dynamixel.move(ID, input_position);
    // Dynamixel.move(2, 400);

  }
}

void print_menu(int sel){

  switch (sel){
    case 1:
      Serial.println("'m' for moving testing");
      Serial.println("'#' for set ID");
      Serial.println("'T' for set torque limits");
      Serial.println("'a' for set angle limits");
      Serial.println("'c' for set current limits");
      Serial.println("'v' for set voltage limits");
      Serial.println("'t' for set temperature limits");

      Serial.println("Choose which mode:");
      break;

    case 2:
      Serial.println("Enter -1' to go back to menu");
      break;

  }
}

//This function tests the input value with know value possible ranges and returns
//if the input_value is valid based on the value selected (value_sel).
int check_value(int value_selected, int input_value){

  switch (value_selected){
    case 1: //Valid ID range
      Serial.println("Choose which mode:");
      break;

    case 2: //Valid servo position range
      Serial.println("Enter -1' to go back to menu");
      break;
  }
}


//used when setting the new servo id.
//should move the servo with the new ID and check if it moved from the
//position and alert the user if it was successful
void test_servoID(int ID){
  current_position = Dynamixel.readPosition(ID);
  new_position = random(450,650);
  Dynamixel.move(newID, new_position);
  if(Dynamixel.moving(ID)){
    Serial.println();
    Serial.print("Servo '");
    Serial.print(ID);
    Serial.print("' is moving from position '");
    Serial.print(current_position);
    Serial.print("' to position '");
    Serial.print(new_position);
    Serial.println("'.");
    Serial.println();
    Serial.println();
  }
}
