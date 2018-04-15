//Imported ibraries
#include <DynamixelSerial1.h>


//Drive mode angles
//0 358 1 512 2 600 3 664 4 512 5 409 6 512 7 700 8 200 9 512 10 323 11 825 12 664 13 500 14 600 15 358 16 512 17 1000
//0 358 1 426 2 512 3 664 4 596 5 512 6 512 7 700 8 200 9 512 10 323 11 825 12 664 13 426 14 512 15 358 16 596 17 512

//Default walking angles

//Global variables
int newID, torque_map, current_position, new_position, id;
int ID =1, torque_level = 512, input_position=500, user_input = 0;
bool menu_printed;
char menu_mode;
int c=0;//counter for debugging

//Function prototypes
void print_message(int sel);
int check_value(int value_selected, int input_value);
void test_servoID(int ID);
void setup_torque(int torque);


//Initalize all parameters and variables
void setup()
{
  Serial.begin(9600);          // Begin Serial Comunication for serial monitor
  Dynamixel.begin(1000000,2);  // Initialize the servo at 1Mbps and Pin Control 2
  menu_printed = false;
  randomSeed(analogRead(0));   // seed the random function that is used for servo ID testing
  setup_torque(torque_level);  //set a default torque value
  while (!Serial){} // wait for serial port to connect. Needed for native USB port only
  delay(1000);
}


void loop()
{
  //check Start of menu if the menu has been printed already
  if (!menu_printed)
  {
    print_message(1);
    menu_printed = true;
  }

//if serial input is availble read it to enter into a menu mode
  if (Serial.available()>0)
  {
    menu_mode = Serial.read();
    Serial.println(menu_mode);
    switch (menu_mode)
    {
      case 'm':    //Moving mode
        Serial.println();
        Serial.println();
        Serial.println("_____Moving mode_____");
        Serial.println("Enter in the form of 'ID Position'. Can string together multiple commands.");
        print_message(2);
        while(ID > -1)
        {
          if (Serial.available()>0)
          {
            ID = Serial.parseInt();
            input_position = Serial.parseInt();
            if (ID > -1 && input_position > 0)
            {
              Serial.print("ID = ");
              Serial.print(ID);
              Serial.print(" ");
              Serial.print("Input position = ");
              Serial.println(input_position);
              Dynamixel.move(ID, input_position);
              //while(Dynamixel.moving(ID)){}
            }
            else{break;}
          }
        }
        break;
      case 's':    //Status of all servos
          Serial.println();
          Serial.println();
          Serial.println("_____Return position mode_____");
          print_message(2);
          while(user_input != -1)
          {
              for(id=0;id<18;id++)
              {
//                Serial.print("'");
                Serial.print(id);
                Serial.print("@");
                Serial.print(Dynamixel.readPosition(id));
                Serial.print("|");
              }
              Serial.println();
              id=0; //reset id to reread the servo positions
              if(Serial.available())
              {
                user_input=Serial.parseInt();
              }
          }
          break;
      case '#':    // Mode to set ID numbers
        Serial.println("_____Set ID mode_____");
        Serial.println("Enter in form of 'old_ID new_ID'.");
        print_message(2);
        while(ID > -1)
        {
          if (Serial.available()>0)
          {
            ID = Serial.parseInt();
            newID = Serial.parseInt();
            if (ID > -1 && newID > -1)
            {
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
      case 'T':    // Torque limits mode
        Serial.println("_____Torque limits mode_____");
        Serial.println("Enter in form of 'New_Torque_Limit'.");
        print_message(2);
       //Serial.println("Not Ready Yet");
        while(torque_level > -1)
        {
          if (Serial.available()>0)
          {
              torque_level = Serial.parseInt();
              if(torque_level >-1)
              {
                setup_torque(torque_level);
                Serial.print("Servo Max torque changed to ");
                torque_map = map(torque_level, 0, 1023, 0, 100);
                Serial.print(torque_map);
                Serial.print("%(");
                Serial.print(torque_level);
                Serial.println(").");
              }
          }
        }
        break;
      case 'a':    // Angle limits mode
        Serial.println("_____Angle limits mode_____");
        Serial.println("Not Ready Yet");
        print_message(2);
        break;
      case 'c':    // Current limits mode
        Serial.println("_____Current limits mode_____");
        Serial.println("Not Ready Yet");
        print_message(2);
        break;
      case 'v':    // Voltage limits mode
        Serial.println("_____Voltage limits mode_____");
        Serial.println("Not Ready Yet");
        print_message(2);
        //Dynamixel.setVoltageLimit(1,65,120);  // Set Operating Voltage from 6.5v to 12v
        break;
      case 't':    // Temperature limits mode
        Serial.println("_____Temperature limits mode_____");
        Serial.println("Not Ready Yet");
        print_message(2);
        // Dynamixel.setTempLimit(1,80);  // Set Max Temperature to 80 Celcius
        break;
      default:
        Serial.println("Please give a valid menu input");
        while (Serial.available())
        {
          menu_mode = Serial.read();
        }
    }
    //reset menu flags & ID(used as a flag as well)
    menu_printed = false;
    user_input = 0;
    ID = 1;
    torque_level = 512;
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
  }
}

//different printout messages to reference when creating the menu
void print_message(int sel)
{
  switch (sel)
  {
    case 1:
      Serial.println("'m' for moving testing");
      Serial.println("'s' for current servo positions");
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

    case 3:
      Serial.println("Value enter Invalid. Must be between 0-252 for a valid ID.");
      break;

    case 4:
      Serial.println("Value enter Invalid. Must be between 0-1023 for a valid ID.");
      break;

  }
}

//This function tests the input value with know value possible ranges and returns
//if the input_value is valid based on the value selected (value_sel).
int check_value(int value_selected, int input_value){

  switch (value_selected){
    case 1: //Valid ID range
      if(value_selected >= 0 && value_selected <= 252)
      {
        return 1;
      }
      else{
        print_message(3);
        return 0;
      }
      break;

    case 2: //Valid servo position range
      if(value_selected >= 0 && value_selected <= 1023)
      {
        return 1;
      }
      else{
        print_message(4);
        return 0;
      }
      break;
  }
}


//used when setting the new servo id.
//should move the servo with the new ID and check if it moved from the
//position and alert the user if it was successful
void test_servoID(int ID)
{
  current_position = Dynamixel.readPosition(ID);
  new_position = random(450,650);
  Dynamixel.move(newID, new_position);
  if(Dynamixel.moving(ID))
  {
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

void setup_torque(int torque)
{
  for (id=0; id<18;id++)
  {
  Dynamixel.setMaxTorque(id,torque);
  }
}
