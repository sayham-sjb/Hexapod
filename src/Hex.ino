#include "avr/wdt.h"
#include "Servo.h"


//-----Servo Characteristics-----
//
//Wires:
//
// red = positive
// brown = negative
// orange = signal
//
//Tested lower and upper bounds:
//
// servo01 0-180
// servo02 25-180
// servo03 10-180
//
//Code will only ask for angles between 45-145 regardless (for now)
//
//Arduino pins used:
//
// servo1 = pin 2
// servo2 = pin 3
// servo3 = pin 4


//-----Prototypes-----
void getAngle(int selected_servo);

void setAngle(char incoming_serial, int selected_servo);

void selectServo(char incoming_serial);

//Defines and sets appropriate values in angle array given a preset motion number
void motionPreset(int angle[], int servo_status[], int motion_preset);

//Receives angle array from motionPreset, movement may not be required by each individual servo
int motion(int angle[], int servo_status[], int servo_speed);

void softwareReboot(void);


//-----Variables-----
//Stored character values from serial buffer
char incoming_serial;

//Additional timer for comparing against main timer
unsigned long servo_time = 0; 

//Robot continues executing a motion every "servo_speed" milliseconds until motion is complete
//ISSUE - should this really be an int?
int servo_speed = 25; 

//Robot moves each servo "servo_interval" number of
//degrees until executed motion is complete
int servo_interval = 32; 

//This is used when not using the potentiometer
int stored_interval = 32; 

//What's left after the distance a servo will travel is divided by interval
int servo_remainder = 0; 

int selected_servo = 0;

//Motion preset number
int motion_preset = 0; 

//ISSUE - zero indexed for convenience (needs to be undone for readability)
int servo_quantity = 2; 

//Define main array of angles and array of status codes for all 18 servos
//
//ISSUE - Does these have to be global? I think they should be since they really are globally applicable values.
//        I think some other variable should be reviewed to determine if they really need to be global though.
//        If these two arrays are global, should I really need to be passing references to them between functions?
//
//ISSUE - Could either of these be char or byte instead for efficiency? (more so servo_status)
//        Arrays are passed by reference by default, so that helps, but if I'm going to convert status to String anyway
//        I should consider the smallest variable type
//
int angle[18] = {90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90};
int servo_status[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//Servo status code definitions
//...pending...

//Temporary strings for holding char values from serial buffer
String str_angle = "";
String str_speed = "";
String str_select = "";
String str_interval = "";
String str_motion_preset = "";

//Used for conditionally reading serial data and executing corresponding functions
boolean incoming_motion = false;
boolean incoming_angle = false;
boolean incoming_speed = false;
boolean incoming_select = false;
boolean incoming_interval = false;

//Declare main array for all 18 servos
Servo servo[18];

//Potentiometer related variables
int pot_pin = 2;
int pot_val = 0;
boolean use_pot = true;


//-----Main-----
void setup()
{
  //Allow time for initialization and create serial connection
  delay(500);
  Serial.begin(9600);
  delay(500);
  
  //Attach servos to pins
  servo[0].attach(2);
  servo[1].attach(3);
  servo[2].attach(4);
  delay(500);
  
  //Manually set servos to normal stance (independent from preset motions)
  servo[0].write(90);
  servo[1].write(115);
  servo[2].write(100);
  delay(500);

  Serial.println("Ready. ");
}


//-----Loop-----
void loop()
{
  //While there are characters in the serial buffer...
  while (Serial.available() > 0)
  {
    //...read the characters from the serial buffer
    incoming_serial = Serial.read();  

    //Get angle of currently selected servo
    if (incoming_serial == 'g')
    {
      getAngle(selected_servo);
    }
    //"Knock" servo (move 5 degrees and return for visual identification)
    else if (incoming_serial == 'k')
    {
      selectServo(incoming_serial);
    }
    //Reset Arduino
    else if (incoming_serial == 'r')
    {
      softwareReboot();
    }
    //Previous or next servo
    else if (incoming_serial == ',' || incoming_serial == '.')
    {      
      selectServo(incoming_serial);
    }
    //Detach selected servo, currently requires reset to re-attach
    else if (incoming_serial == 'd')
    {
      servo[selected_servo].detach();
    }
    //Enable or disable reading potentiometer for interval value
    else if (incoming_serial == 'p')
    {
      if (use_pot == true)
      {
        use_pot = false;
        servo_interval = stored_interval;
        Serial.println("Potentiometer is OFF");
        Serial.println("servo_interval reverted to: ");
        Serial.println(servo_interval);
      }
      else
      {
        use_pot = true;
        Serial.println("Potentiometer is ON");
        Serial.println("servo_interval is read as: ");
        Serial.println((analogRead(pot_pin))/32);
      }
    }
    else if (incoming_serial == '+' || incoming_serial == '-')
    {
      setAngle(incoming_serial, selected_servo);
    }
    //ANGLE - Receive a three digit numeric angle
    else if ((isDigit(incoming_serial)) && (incoming_angle == true))
    {
      //***debug***
      Serial.println("...str_angle was: ");
      Serial.println(str_angle);
      Serial.println("");

      //***debug***
      Serial.println(incoming_serial);
      Serial.println("...is digit");

      str_angle += incoming_serial;

      //***debug***
      Serial.println("...str_angle IS NOW: ");
      Serial.println(str_angle);

      if (str_angle.length() == 3)
      {
        Serial.println("str_angle IS length 3");

        //ISSUE
        //*** Update 3/8/15 ***
        //I'm not sure why I wrote the below commentary, it seems like
        //char would be preferable.  Review when possible.
        //
        //*** Previous commentary ***
        //This is a work around to temporarily avoid rewriting the program with incoming_serial as String,
        //I'm just going to call setAngle three times until all digits of manual angle have
        //been copied, then let setAngle decode it...
        //Eventually, will use Serial.readString() and/or Serial.readStringUntil('\n') to fix this.
        //There is also Serial.parseInt();
        //
        char _cheat = 'Y';
        setAngle(_cheat, selected_servo);
        incoming_angle = false;
        
        //***debug***
        Serial.println("incoming_angle is: ");
        Serial.println(incoming_angle);
        str_angle = "";
      }
    }
    //SPEED - Receive a three digit speed in milliseconds
    else if ((isDigit(incoming_serial)) && (incoming_speed == true))
    {
      //***debug***
      Serial.println("...str_speed was: ");
      Serial.println(str_speed);
      Serial.println("");

      //***debug***
      Serial.println(incoming_serial);
      Serial.println("...is digit");

      str_speed += incoming_serial;

      //***debug***
      Serial.println("...str_speed IS NOW: ");
      Serial.println(str_speed);
      
      if (str_speed.length() == 3)
      {
        Serial.println("str_speed IS length 3");
        incoming_speed = false;

        //***debug***
        Serial.println("str_speed is: ");
        Serial.println(str_speed);
        Serial.println("");
        servo_speed = str_speed.toInt();
        Serial.println("servo_speed is now: ");
        Serial.println(servo_speed);
        str_speed = "";
      }
    }
    //SELECT - Receive a two digit servo selection
    else if ((isDigit(incoming_serial)) && (incoming_select == true))
    {
      //***debug***
      Serial.println(incoming_serial);
      Serial.println("...is digit");
      
      str_select += incoming_serial;

      //***debug***
      Serial.println("...str_select IS NOW: ");
      Serial.println(str_select);
      
      if (str_select.length() == 2)
      {
        if (str_select.toInt() <= 2)
        {
          selected_servo = str_select.toInt();

          //***debug***
          Serial.println("selected_servo IS NOW: ");
          Serial.println(selected_servo);
          incoming_select = false;
          str_select = "";
        }
        else
        {
          Serial.println("Invalid servo selection.");
          incoming_select = false;
          str_select = "";
        }
      }      
    }
    else if (incoming_serial == 'a')
    {
      incoming_angle = true;

      //***debug***
      Serial.println("incoming_angle is: ");
      Serial.println(incoming_angle);
    }
    else if (incoming_serial == 'j')
    {
      incoming_select = true;
    }
    //MOTION - Receive a two digit motion number
    else if (isDigit(incoming_serial) && (incoming_motion == true))
    {
      //***debug***
      Serial.println(incoming_serial);
      Serial.println("...is digit");

      str_motion_preset += incoming_serial;

      //***debug***
      Serial.println("...str_motion_preset IS NOW: ");
      Serial.println(str_motion_preset);

      if (str_motion_preset.length() == 2)
      {
        motion_preset = str_motion_preset.toInt();

        //***debug***
        Serial.println("motion_preset is: ");
        Serial.println(motion_preset);

        //Call motionPreset function, pass angle array, servo_status array, motion_preset number, validity checked within motionPreset
        motionPreset(angle, servo_status, motion_preset); 
        incoming_motion = false;
        str_motion_preset = "";
      } 
    }
    else if (incoming_serial == 'm')
    {
      incoming_motion = true;
    }
    else if (incoming_serial == '[')
    {
      servo_speed -= 1;
      Serial.println("Servo speed is: ");
      Serial.println(servo_speed);
    }
    else if (incoming_serial == ']')
    {
      servo_speed += 1;
      Serial.println("Servo speed is: ");
      Serial.println(servo_speed);
    }
    else if (incoming_serial == 's')
    {
      incoming_speed = true;
    }
    //INTERVAL - Receive a two digit interval number
    else if ((isDigit(incoming_serial)) && (incoming_interval == true))
    {
      //***debug***
      Serial.println(incoming_serial);
      Serial.println("...is digit");
      
      str_interval += incoming_serial;
      //***debug***
      Serial.println("...str_interval IS NOW: ");
      Serial.println(str_interval);
      
      if (str_interval.length() == 2)
      {
        if (str_interval.toInt() <= 32)
        {
          use_pot = false;

          //***debug***
          Serial.println("servo_interval was: ");
          Serial.println(servo_interval);
          Serial.println("");
          
          servo_interval = str_interval.toInt();
          stored_interval = servo_interval;

          //***debug***
          Serial.println("Potentiometer is: ");
          Serial.println(use_pot);
          Serial.println("");
          Serial.println("servo_interval IS NOW: ");
          Serial.println(servo_interval);
          Serial.println("");
          incoming_interval = false;
          str_interval = "";
        }
        else
        {
          Serial.println("Invalid interval selection.");
          incoming_interval = false;
          str_interval = "";
        }
      }      
    }
    else if (incoming_serial == 'i')
    {
      incoming_interval = true;
    }
    else //If incoming_serial isn't something expected...
    {
      Serial.println("Invalid request.");
    }
  }
}


//-----Get Angle Function-----
void getAngle(int selected_servo)
{ 
  int _angle = servo[selected_servo].read();
  Serial.println(_angle);
};


//-----Select Servo Function-----
void selectServo(char incoming_serial)
{ 
  if (incoming_serial == ',')
  {
    if (selected_servo == 0)
    {
      selected_servo = servo_quantity;
    }
    else
    {
      selected_servo -= 1;
    }
  }
  else if (incoming_serial == '.')
  {      
    if (selected_servo == servo_quantity)
    {
      selected_servo = 0;
    }
    else
    {
      selected_servo += 1;
    } 
  }
  
  int _a = servo[selected_servo].read();
  if (_a >= 45 && _a <= 140)
  {
    Serial.println("Before move: ");
    Serial.println(_a);
    
    servo[selected_servo].write(_a + 5);
    _a += 5;
    Serial.println("After move: ");
    Serial.println(_a);
    
    delay (250);
    
    servo[selected_servo].write(_a - 5);
    _a -= 5;
    Serial.println("After move back: ");
    Serial.println(_a);
  }
  else if (_a = 145)
  {
    servo[selected_servo].write(_a - 5);
    _a -= 5;
    delay (250);
    servo[selected_servo].write(_a + 5);
    _a += 5;
  }

  //***debug***
  Serial.println("Selected servo: ");
  Serial.println(selected_servo);
  delay (250);
};


//-----Set Angle Function-----
void setAngle(char incoming_serial, int selected_servo)
{
  if (incoming_serial == '+' || incoming_serial == '-')
  {
    int _angleA = servo[selected_servo].read();
    int _angleB;
    
    if (incoming_serial == '+')
    {  
      _angleB = _angleA + 5;
    }
    else if (incoming_serial == '-')
    {
      _angleB = _angleA - 5;
    }
    
    if (_angleB >= 45 && _angleB <= 145)
    {
      //Move servo to the new angle
      servo[selected_servo].write(_angleB);

      //***debug***
      Serial.println(_angleB);   
    }
    else
    {
      Serial.println("Out of bounds. ");
    }
  }
  else if (incoming_serial = 'Y')
  {
    int _a = str_angle.toInt();
    
    if (_a <= 145 && _a >= 45)
    {
      Serial.println("Moving to: ");
      Serial.println(str_angle);
      Serial.println("");
      servo[selected_servo].write(_a);
      incoming_angle = false;
      str_angle = "";

      //***debug***
      Serial.println(str_angle);
    }
    else
    {
      Serial.println(str_angle);
      Serial.println("is out of bounds. ");
      incoming_angle = false;
      str_angle = "";
    }
  }
  else
  {
    Serial.println("Undefined angle. ");
    incoming_angle = false;
    str_angle = "";
  }
};


//-----Motion Preset Function-----
//Updates the appropriate values in the angle array given a motion preset number and calls the motion function
void motionPreset(int angle[], int servo_status[], int motion_preset)
{  
  //*** debug ***
  Serial.println("Received motion number: ");
  Serial.println(motion_preset);

  //Motion preset definitions
  switch (motion_preset)
  {
    case 0: //TEST
      angle[0] = 90;
      angle[1] = 90;
      angle[2] = 90;
      break;
    case 1: //NORMAL
      angle[0] = 90;
      angle[1] = 115;
      angle[2] = 100;

      //***debug***
      Serial.println("CASE 1 - sending angles: ");
      Serial.println(angle[0]);
      Serial.println(angle[1]);
      Serial.println(angle[2]);

      break;
    case 2: //WALK - Pick leg up all the way
      angle[0] = 90;
      angle[1] = 145;
      angle[2] = 145;

      //***debug***
      Serial.println("CASE 2 - sending angles: ");
      Serial.println(angle[0]);
      Serial.println(angle[1]);
      Serial.println(angle[2]);

      break;
    case 3: //WALK - Move leg forward all the way
      angle[0] = 145;
      angle[1] = 145;
      angle[2] = 145;

      //***debug***
      Serial.println("CASE 3 - sending angles: ");
      Serial.println(angle[0]);
      Serial.println(angle[1]);
      Serial.println(angle[2]);

      break;
    case 4: //WALK - Put leg down to ground only (this still isn't the normal stance position)
      angle[0] = 145;
      angle[1] = 115;
      angle[2] = 100;

      //***debug***
      Serial.println("CASE 4 - sending angles: ");
      Serial.println(angle[0]);
      Serial.println(angle[1]);
      Serial.println(angle[2]);

      break;
    case 5: //TEST
      angle[0] = 45;
      angle[1] = 145;
      angle[2] = 45;

      //***debug***
      Serial.println("CASE 5 - sending angles: ");
      Serial.println(angle[0]);
      Serial.println(angle[1]);
      Serial.println(angle[2]);

      break;
    case 6: //TEST
      angle[0] = 145;
      angle[1] = 145;
      angle[2] = 45;

      //***debug***
      Serial.println("CASE 6 - sending angles: ");
      Serial.println(angle[0]);
      Serial.println(angle[1]);
      Serial.println(angle[2]);

      break;
    case 7: //TEST
      angle[0] = 145;
      angle[1] = 45;
      angle[2] = 45;

      //***debug***
      Serial.println("CASE 7 - sending angles: ");
      Serial.println(angle[0]);
      Serial.println(angle[1]);
      Serial.println(angle[2]);

      break;
    case 8: //TEST
      angle[0] = 45;
      angle[1] = 45;
      angle[2] = 45;

      //***debug***
      Serial.println("CASE 8 - sending angles: ");
      Serial.println(angle[0]);
      Serial.println(angle[1]);
      Serial.println(angle[2]);

      break;
    default:
      Serial.println("Undefined motion.");
  }

  //Update servo_status array to indicate servos are in motion
  servo_status[0] = 1;
  servo_status[1] = 1;
  servo_status[2] = 1;

  //Call motion function using angles applied by motionPreset
  motion(angle, servo_status, servo_speed);
};


//-----Motion Preset Loops -----
//...pending...


//-----Motion Function-----
int motion(int angle[], int servo_status[], int servo_speed)
{
  //Get the current run-time in milliseconds and store it
  servo_time = millis();
  
  for (int a=0; a<=2; a++)
  {    
    if (angle[a] < 45 || angle[a] > 145)
    {
      return 1;
    }
    Serial.println("Received angle: ");
    Serial.println(angle[a]);
  }
  
  boolean motion_complete = false;

  while (motion_complete == false)
  {
    //*** debug ***
    //Serial.println("Is millis - servo_time >= servo_speed? ");
    //If it has been longer than servo_speed (ex: 25ms)...
    if ((millis()-servo_time) >= servo_speed)
    {
      //*** debug***
      //Serial.println("...yes, continuing motion.");
      
      //Save time reference for next update 
      servo_time = millis(); 
      
      //For all servos...
      for (int b=0; b<=2; b++)  
      { 
        //If using the potentiometer then read the value,
        //otherwise servo_interval has already been set by user
        //or by default
        if (use_pot)
        {
          pot_val = analogRead(pot_pin);
          
          if (pot_val < 32)
          {
            servo_interval = 1;
          }
          else
          {
            servo_interval = pot_val / 32;
          }
        }

        //Determine the distance left to move
        int _a = angle[b] - servo[b].read();
        servo_remainder = abs(_a);
        
        //*** debug ***
        //Serial.println("servo: ");
        //Serial.println(b);
        //Serial.println("remaining degrees: ");
        //Serial.println(servo_remainder);
        //Serial.println("motion_tally: ");
        //Serial.println(motion_tally);
        //Serial.println("of: ");
        //Serial.println(servo_interval);

        //Finalize movement (remainder is less than interval)
        if (servo_interval > servo_remainder && servo_remainder != 0)
        {
          servo[b].write(angle[b]);
        }

        //Continue movement by addition
        else if (angle[b] > servo[b].read())
        {
          servo[b].write(servo[b].read() + servo_interval);    
        }

        //Continue movement by subtraction
        else if (angle[b] < servo[b].read())
        {
          servo[b].write(servo[b].read() - servo_interval);
        }

        //Update servo_status array as appropriate to indicate servos are no longer in motion
        else
        {
          servo_status[b] = 0;
        }
      }

      //Check servo status to determine if all servos are finished moving (meaning motion is complete)
      //
      //ISSUE - does not take all 18 servos into account, there is no plan for overlapping motions yet,
      //        can't expect status 0 for all 18 servos just because a motion involving 3 of them is complete,
      //        this check will at a minimum need to be repositioned
      if ((servo_status[0] == 0) && (servo_status[1] == 0) && (servo_status[2] == 0))
      {
        motion_complete = true;
      }
    }
  }

  Serial.println("Motion complete.");

  //***debug***
  Serial.println("angles now read as: ");
  Serial.println(servo[0].read());
  Serial.println(servo[1].read());
  Serial.println(servo[2].read());

  //Return motion_complete to default state
  motion_complete = false;
};


//----Software Reboot Function----
void softwareReboot(void)
{
  wdt_enable(WDTO_500MS);
  while(1)
  {
  }
}