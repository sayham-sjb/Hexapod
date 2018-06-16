#include <math.h>
#include <Servo.h>
#include <string.h>

#define logUSB 1
#define logBT 0

#define RRearCoxaServo 0
#define RRearFemurServo 1
#define RRearTibiaServo 2
#define RCenterCoxaServo 3
#define RCenterFemurServo 4
#define RCenterTibiaServo 5
#define RFrontCoxaServo 6
#define RFrontFemurServo 7
#define RFrontTibiaServo 8
#define LRearCoxaServo 9
#define LRearFemurServo 10
#define LRearTibiaServo 11
#define LCenterCoxaServo 12
#define LCenterFemurServo 13
#define LCenterTibiaServo 14
#define LFrontCoxaServo 15
#define LFrontFemurServo 16
#define LFrontTibiaServo 17


struct RobotStatus {
  int GaitType;  
  //Init position of the leg
  int RFGaitLegNr;          
  int RMGaitLegNr;          
  int RRGaitLegNr;          
  int LFGaitLegNr;          
  int LMGaitLegNr;          
  int LRGaitLegNr; 
  boolean LastLeg;
  
  int StepsInGait; // Nr of steps in gait
  int TLDivFactor; // Nr of steps that a leg is on the floor while waling
  boolean HalfLiftHeight; // does the step sequence has half lifted legs
  double LegLiftHeight;     //Current Travel height
  int TravelSpeed;       //Current Move Speed [ms]
  double TravelLengthX;     //Current Travel length X (sideways)
  double TravelLengthZ;     //Current Travel length Z (front - back)
  double TravelRotationY;   //Current Travel Rotation Y
  
  boolean IsOffRoadLiftHeight; //Allows user to switch between off-road and on-road step heights
  int CycleLegs;
  
  // Body position
  double BodyPosX;      //Global Input for the position of the body
  double BodyPosY;      //Global Input for the position of the body
  double BodyPosZ;      //Global Input for the position of the body
        
        //Body Inverse Kinematics
  double BodyRotX;      //Global Input pitch of the body
  double BodyRotY;      //Global Input rotation of the body
  double BodyRotZ;      //Global Input roll of the body
  double BodyIKPosX;    //Output Position X of feet with Rotation
  double BodyIKPosY;    //Output Position Y of feet with Rotation
  double BodyIKPosZ;    //Output Position Z of feet with Rotation

        //Leg Inverse Kinematics
  boolean IKSolution;        //Output true if the solution is possible
  boolean IKSolutionWarning;  //Output true if the solution is NEARLY possible
  boolean IKSolutionError;   //Output true if the solution is NOT possible
  double IKFemurAngle;  //Output Angle of Femur in degrees
  double IKTibiaAngle;  //Output Angle of Tibia in degrees
  double IKCoxaAngle;   //Output Angle of Coxa in degrees
  
  // Actual positions, messured in mm from the coxa offset to the tips of the feet
  double  RFPosX;           //Actual Position of the Right Front Leg
  double  RFPosY;
  double  RFPosZ;
  double  RMPosX;          //Actual Position of the Right Middle Leg
  double  RMPosY;
  double  RMPosZ;
  double  RRPosX;           //Actual Position of the Right Rear Leg
  double  RRPosY;
  double  RRPosZ;
  double  LFPosX;           //Actual Position of the Left Front Leg
  double  LFPosY;
  double  LFPosZ;
  double  LMPosX;          //Actual Position of the Left Middle Leg
  double  LMPosY;
  double  LMPosZ;
  double  LRPosX;           //Actual Position of the Left Rear Leg
  double  LRPosY;
  double  LRPosZ;  
  
  // Angles
  double*  RFCoxaAngle;       //Actual Angle of the Right Front Leg
  double*  RFFemurAngle;
  double*  RFTibiaAngle;

  double*  RMCoxaAngle;       //Actual Angle of the Right Middle Leg
  double*  RMFemurAngle;
  double*  RMTibiaAngle;

  double*  RRCoxaAngle;       //Actual Angle of the Right Rear Leg
  double*  RRFemurAngle;
  double*  RRTibiaAngle;

  double*  LFCoxaAngle;       //Actual Angle of the Left Front Leg
  double*  LFFemurAngle;
  double*  LFTibiaAngle;

  double*  LMCoxaAngle;       //Actual Angle of the Left Middle Leg
  double*  LMFemurAngle;
  double*  LMTibiaAngle;

  double*  LRCoxaAngle;       //Actual Angle of the Left Rear Leg
  double*  LRFemurAngle;
  double*  LRTibiaAngle;
  
  double GaitStep;          //Global Input Gait step
  double TravelMulti;       //Multiplier for the length of the step
  
  
  // Relative position corresponding to the Gait
  double RFGaitPosX;        
  double RFGaitPosY;
  double RFGaitPosZ;
  double RFGaitRotY;        //Relative rotation corresponding to the Gait

  double RMGaitPosX;
  double RMGaitPosY;
  double RMGaitPosZ;
  double RMGaitRotY;

  double RRGaitPosX;
  double RRGaitPosY;
  double RRGaitPosZ;
  double RRGaitRotY;

  double LFGaitPosX;
  double LFGaitPosY;
  double LFGaitPosZ;
  double LFGaitRotY;

  double LMGaitPosX;
  double LMGaitPosY;
  double LMGaitPosZ;
  double LMGaitRotY;

  double LRGaitPosX;
  double LRGaitPosY;
  double LRGaitPosZ;
  double LRGaitRotY;
  
};




// Coxa, Femur, Tibia of   RR          RC          RF          LR          LC          LF
const int servoPins[18] = {56, 55, 54, 51, 50, 49, 48, 47, 46, 11, 12, 13, 24, 25, 26, 27, 28, 29};

Servo servos[18];

int servoTimingDirection[18] = {   -1, -1, -1, 
                                   -1,  -1, -1, 
                                   -1, -1, -1,
                                  1,  1, 1,
                                  1, 1, 1,
                                   1,  1,  1}; 
                                   
int servoAngleDirection[18] = {   -1, -1, -1, 
                                   -1,  -1, -1, 
                                   -1, -1, -1,
                                  1,  1, 1,
                                  1, 1, 1,
                                   1,  1,  1};

// Settings                                   
int servoAngleOffset[18] = {     -93, -93, -91, 
                                 -83, -85, -79, 
                                 -90, -86, -87,
                                 -87, -98, -104,
                                 -83, -99, -105,
                                 -72, -93, -100};                                    
                                   
double servoAngleRanges[18][2] = {  {-45, 45}, {-55, 65}, {-70, 90}, // R Rear OK
                                    {-45, 45}, {-70, 75}, {-70, 90}, // R Center tibia servo not working
                                    {-45, 45}, {-55, 75}, {-70, 90}, // R Front ok
                                    {-45, 45}, {-55, 65}, {-65, 90}, // L Rear tibio servo sometimes wrong
                                    {-45, 45}, {-70, 75}, {-70, 90}, // L Center OK
                                    {-45, 45}, {-55, 75}, {-70, 90}}; // L Front coxa servo not working
// Body dimensions (in mm)
double CoxaLength = 22;       //Length of the Coxa [mm]
double FemurLength = 100;      //Length of the Femur [mm]
double TibiaLength = 145;     //Lenght of the Tibia [mm]
double WakeStandingHeightY = 50; //Typical stading height (body above ground) in [mm]

double RFOffsetX = 57;       //Distance X from center of the body to the Right Front coxa
double RFOffsetZ = -103;       //Distance Z from center of the body to the Right Front coxa
double RMOffsetX = 76;       //Distance X from center of the body to the Right Middle coxa
double RMOffsetZ = 0;         //Distance Z from center of the body to the Right Middle coxa
double RROffsetX = 57;       //Distance X from center of the body to the Right Rear coxa
double RROffsetZ = 103;        //Distance Z from center of the body to the Right Rear coxa

double LFOffsetX = -57;        //Distance X from center of the body to the Left Front coxa
double LFOffsetZ = -103;       //Distance Z from center of the body to the Left Front coxa
double LMOffsetX = -76;        //Distance X from center of the body to the Left Middle coxa
double LMOffsetZ = 0 ;        //Distance Z from center of the body to the Left Middle coxa
double LROffsetX = -57;        //Distance X from center of the body to the Left Rear coxa
double LROffsetZ = 103;        //Dista

double RFCoxaOrrientation = 45;
double RMCoxaOrrientation = 0;
double RRCoxaOrrientation = -45;
double LFCoxaOrrientation = 135;
double LMCoxaOrrientation = 180;
double LRCoxaOrrientation = -135;
double TibiaOffset = 90;                                    
                                    
double servoAngleFactors[18];                                    
double servoAngles[18];
double servoPrevAngles[18];

// serial data
String inputString = ""; // string to hold incoming data
boolean stringComplete = false;
bool moving = true;

RobotStatus Status;

void setup()
{   
  Serial.begin(115200);
  //Serial.begin(9600);
  inputString.reserve(100); 
  
  Serial1.begin(9600);
  
  initializeValues();
  attachServos(); 
}
  
void initializeValues()
{
  for (int i=0;i<18;i++)
  {
    servoPrevAngles[i] = -200;
    servoAngles[i] = 0;
    servoAngleFactors[i] = servoTimingDirection[i] * 10;
  } 
  
  //Feet Positions
  //Measured in mm from the coxa offset of the robot to the tips of the feet)
  Status.RFPosX = 65;           
  Status.RFPosY = -35;
  Status.RFPosZ = -65;
  Status.RMPosX = 92;        
  Status.RMPosY = -35;
  Status.RMPosZ = 0;
  Status.RRPosX = 65;          
  Status.RRPosY = -35;
  Status.RRPosZ = 65;
  Status.LFPosX = -65;          
  Status.LFPosY = -35;
  Status.LFPosZ = -65;
  Status.LMPosX = -92;        
  Status.LMPosY = -35;
  Status.LMPosZ = 0;
  Status.LRPosX = -65;          
  Status.LRPosY = -35;
  Status.LRPosZ = 65;  
  
  Status.GaitType = 0;
  // Body position
  Status.BodyPosY = 0;
  Status.BodyPosX = 0;
  Status.BodyPosZ = 0;
  Status.BodyRotY = 0;
  Status.BodyRotX = 0;
  Status.BodyRotZ = 0;
  // Movements
  Status.TravelLengthX = 0;
  Status.TravelLengthZ = 0;
  Status.TravelRotationY = 0;
  
  //Body Positions
  Status.BodyPosX = 0;
  Status.BodyPosY = 0;
  Status.BodyPosZ = 0;

  //Body Rotations
  Status.BodyRotX = 0;
  Status.BodyRotY = 0;
  Status.BodyRotZ = 0;

  //Gait
  Status.LegLiftHeight = WakeStandingHeightY * (2.0 / 3.0);
  Status.GaitStep = 1;
  
  Status.RFCoxaAngle = &servoAngles[RFrontCoxaServo];
  Status.RFFemurAngle = &servoAngles[RFrontFemurServo];
  Status.RFTibiaAngle = &servoAngles[RFrontTibiaServo];
  Status.RMCoxaAngle = &servoAngles[RCenterCoxaServo];
  Status.RMFemurAngle = &servoAngles[RCenterFemurServo];
  Status.RMTibiaAngle = &servoAngles[RCenterTibiaServo];
  Status.RRCoxaAngle = &servoAngles[RRearCoxaServo];
  Status.RRFemurAngle = &servoAngles[RRearFemurServo];
  Status.RRTibiaAngle = &servoAngles[RRearTibiaServo];
  Status.LFCoxaAngle = &servoAngles[LFrontCoxaServo];
  Status.LFFemurAngle = &servoAngles[LFrontFemurServo];
  Status.LFTibiaAngle = &servoAngles[LFrontTibiaServo];
  Status.LMCoxaAngle = &servoAngles[LCenterCoxaServo];
  Status.LMFemurAngle = &servoAngles[LCenterFemurServo];
  Status.LMTibiaAngle = &servoAngles[LCenterTibiaServo];
  Status.LRCoxaAngle = &servoAngles[LRearCoxaServo];
  Status.LRFemurAngle = &servoAngles[LRearFemurServo];
  Status.LRTibiaAngle = &servoAngles[LRearTibiaServo];
  
  // Speed
  Status.TravelSpeed = 1000;
  GaitSelect(0); 
}

void attachServos()
{
  for (int i=0;i<18;i++)
  {
    servos[i].attach(servoPins[i], 600, 2400);
  }  
}

void detachServos()
{
  for (int i=0;i<18;i++)
  {
    servos[i].detach();
  }  
}

void loop() {
  
  // read inputs
  if (stringComplete) {
    logString("SerialInput: ");
    logString(inputString);
    logString("\n");
    int startPos = 0;
    int cmdPos = inputString.indexOf('=', startPos);
    String command = inputString.substring(startPos, cmdPos);
    int endPos = inputString.indexOf(';', startPos);
    String value = inputString.substring(cmdPos+1, endPos);     
    
    if (command == "ver")
    {
      logString("0.1");
      
    } 
   else if  (command == "servoangles")
   {
      double values[18];
      splitString(value, 18, values);      
      double angles[18];
      
      for (int i=0;i<18;i++)
      {
        //Serial.print(i);
        //Serial.print(": ");
        //Serial.print(values[i]);
        //Serial.print(" -> ");
        angles[i] = values[i]; 
        //Serial.println(angles[i]);
      }
      
      setPositions(angles);
   }
   else if  (command == "servotimes")
   {
      double times[18];
      splitString(value, 18, times);
      
      setServoTimes(times);
   }
   else if  (command == "move")
   {
     if (value == "wake")
     {
      wake();
      logString("wake\n ");
     } else if (value == "sleep")
     {
      sleep();
      logString("sleep\n ");
     } 
   } 
   else if  (command == "motion")
   {
      double motion[10];
      splitString(value, 10, motion);
      
      Status.BodyPosY = motion[0];
      Status.BodyPosX = motion[1];
      Status.BodyPosZ = motion[2];
      Status.BodyRotY = motion[3];
      Status.BodyRotX = motion[4];
      Status.BodyRotZ = motion[5];
      Status.TravelLengthX = motion[6];      
      Status.TravelLengthZ = motion[7];
      Status.TravelRotationY = motion[8]; 
      Status.TravelSpeed = motion[9];      
   } 
   else if  (command == "gaittype")
   {
      int type = (int) stringToNumber(value);
      GaitSelect(type);
   } else if  (command == "onroad")
   {
      if (value == "0") // offroad
      {
        Status.IsOffRoadLiftHeight = true;
        Status.LegLiftHeight = Status.BodyPosZ * (1.0 / 3.0);
      }
      else // onroad
      {
        Status.IsOffRoadLiftHeight = true;
        Status.LegLiftHeight = Status.BodyPosZ * (2.0 / 3.0);
      }
   } 
    
    // clear the string:
    inputString = "";
    stringComplete = false;
  } 
  
  // Calculate Angles
  DoInverseKinetics();
  
  // Send servoangles to serial for debug
  LogAngles("calculatedangles");
  
  setPositions(servoAngles);
  
  
  delay(Status.TravelSpeed);
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if ((inChar == '\n' || inChar == ';')  && (inputString.length() > 0))
    {
      stringComplete = true;
    } 
  }
}

void serialEvent1() {
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n' || inChar == ';') {
      stringComplete = true;
    } 
  }
}

void LogAngles(String name)
{
  logString(name);
  logString("=");
  logInt(*Status.RRCoxaAngle); logString(","); logInt(*Status.RRFemurAngle); logString(","); logInt(*Status.RRTibiaAngle); logString(","); 
  logInt(*Status.RMCoxaAngle); logString(","); logInt(*Status.RMFemurAngle); logString(","); logInt(*Status.RMTibiaAngle); logString(","); 
  logInt(*Status.RFCoxaAngle); logString(","); logInt(*Status.RFFemurAngle); logString(","); logInt(*Status.RFTibiaAngle); logString(","); 
  logInt(*Status.LRCoxaAngle); logString(","); logInt(*Status.LRFemurAngle); logString(","); logInt(*Status.LRTibiaAngle); logString(","); 
  logInt(*Status.LMCoxaAngle); logString(","); logInt(*Status.LMFemurAngle); logString(","); logInt(*Status.LMTibiaAngle); logString(","); 
  logInt(*Status.LFCoxaAngle); logString(","); logInt(*Status.LFFemurAngle); logString(","); logInt(*Status.LFTibiaAngle); logString(";\n");  
}
void DoInverseKinetics()
{
  Status.LastLeg = false;
  
  //Calculate Gait Positions
  Gait(Status.LRGaitLegNr, &Status.LRGaitPosX, &Status.LRGaitPosY, &Status.LRGaitPosZ, &Status.LRGaitRotY);
  Gait(Status.RFGaitLegNr, &Status.RFGaitPosX, &Status.RFGaitPosY, &Status.RFGaitPosZ, &Status.RFGaitRotY);
  Gait(Status.LMGaitLegNr, &Status.LMGaitPosX, &Status.LMGaitPosY, &Status.LMGaitPosZ, &Status.LMGaitRotY);
  Gait(Status.RRGaitLegNr, &Status.RRGaitPosX, &Status.RRGaitPosY, &Status.RRGaitPosZ, &Status.RRGaitRotY);
  Gait(Status.LFGaitLegNr, &Status.LFGaitPosX, &Status.LFGaitPosY, &Status.LFGaitPosZ, &Status.LFGaitRotY);
  Status.LastLeg = true;
  Gait(Status.RMGaitLegNr, &Status.RMGaitPosX, &Status.RMGaitPosY, &Status.RMGaitPosZ, &Status.RMGaitRotY);  

  //Right Front Leg
  BodyIK(Status.RFPosX + Status.BodyPosX + Status.RFGaitPosX, Status.RFPosZ + Status.BodyPosZ + Status.RFGaitPosZ, RFOffsetX, RFOffsetZ, Status.RFGaitRotY);
  LegIK(Status.RFPosX + Status.BodyPosX + Status.BodyIKPosX + Status.RFGaitPosX, Status.RFPosY - Status.BodyPosY + Status.BodyIKPosY + Status.RFGaitPosY, Status.RFPosZ + Status.BodyPosZ + Status.BodyIKPosZ + Status.RFGaitPosZ, true);
  *Status.RFCoxaAngle = -1 * (Status.IKCoxaAngle - RFCoxaOrrientation); 
  *Status.RFFemurAngle = Status.IKFemurAngle;
  *Status.RFTibiaAngle = Status.IKTibiaAngle - TibiaOffset;
  
 //Right Middle leg
  BodyIK(Status.RMPosX - Status.BodyPosX + Status.RMGaitPosX, Status.RMPosZ + Status.BodyPosZ + Status.RMGaitPosZ, RMOffsetX, RMOffsetZ, Status.RMGaitRotY);
  LegIK(Status.RMPosX + Status.BodyPosX + Status.BodyIKPosX + Status.RMGaitPosX, Status.RMPosY - Status.BodyPosY + Status.BodyIKPosY + Status.RMGaitPosY, Status.RMPosZ + Status.BodyPosZ + Status.BodyIKPosZ + Status.RMGaitPosZ, true);
  *Status.RMCoxaAngle = -1 * (Status.IKCoxaAngle - RMCoxaOrrientation);
  *Status.RMFemurAngle = Status.IKFemurAngle;
  *Status.RMTibiaAngle = Status.IKTibiaAngle - TibiaOffset;

  //Right Rear leg
  BodyIK(Status.RRPosX + Status.BodyPosX + Status.RRGaitPosX, Status.RRPosZ + Status.BodyPosZ + Status.RRGaitPosZ, RROffsetX, RROffsetZ, Status.RRGaitRotY);
  LegIK(Status.RRPosX + Status.BodyPosX + Status.BodyIKPosX + Status.RRGaitPosX, Status.RRPosY - Status.BodyPosY + Status.BodyIKPosY + Status.RRGaitPosY, Status.RRPosZ + Status.BodyPosZ + Status.BodyIKPosZ + Status.RRGaitPosZ, true);
  *Status.RRCoxaAngle = -1 * (Status.IKCoxaAngle - RRCoxaOrrientation);
  *Status.RRFemurAngle = Status.IKFemurAngle;
  *Status.RRTibiaAngle = Status.IKTibiaAngle - TibiaOffset;

  //Left Front leg
  BodyIK(Status.LFPosX - Status.BodyPosX + Status.LFGaitPosX, Status.LFPosZ + Status.BodyPosZ + Status.LFGaitPosZ, LFOffsetX, LFOffsetZ, Status.LFGaitRotY);
  LegIK(Status.LFPosX + Status.BodyPosX + Status.BodyIKPosX + Status.LFGaitPosX, Status.LFPosY - Status.BodyPosY + Status.BodyIKPosY + Status.LFGaitPosY, Status.LFPosZ + Status.BodyPosZ + Status.BodyIKPosZ + Status.LFGaitPosZ, false);
  *Status.LFCoxaAngle = Status.IKCoxaAngle - LFCoxaOrrientation;
  *Status.LFFemurAngle = Status.IKFemurAngle;
  *Status.LFTibiaAngle = Status.IKTibiaAngle - TibiaOffset;

  //Left Middle leg
  BodyIK(Status.LMPosX - Status.BodyPosX + Status.LMGaitPosX, Status.LMPosZ + Status.BodyPosZ + Status.LMGaitPosZ, LMOffsetX, LMOffsetZ, Status.LMGaitRotY);
  LegIK(Status.LMPosX + Status.BodyPosX + Status.BodyIKPosX + Status.LMGaitPosX, Status.LMPosY - Status.BodyPosY + Status.BodyIKPosY + Status.LMGaitPosY, Status.LMPosZ + Status.BodyPosZ + Status.BodyIKPosZ + Status.LMGaitPosZ, false);
  *Status.LMCoxaAngle = Status.IKCoxaAngle - LMCoxaOrrientation;
  if (*Status.LMCoxaAngle < -180) *Status.LMCoxaAngle += 360;
  *Status.LMFemurAngle = Status.IKFemurAngle;
  *Status.LMTibiaAngle = Status.IKTibiaAngle - TibiaOffset;

  //Left Rear leg
  BodyIK(Status.LRPosX - Status.BodyPosX + Status.LRGaitPosX, Status.LRPosZ + Status.BodyPosZ + Status.LRGaitPosZ, LROffsetX, LROffsetZ, Status.LRGaitRotY);
  LegIK(Status.LRPosX + Status.BodyPosX + Status.BodyIKPosX + Status.LRGaitPosX, Status.LRPosY - Status.BodyPosY + Status.BodyIKPosY + Status.LRGaitPosY, Status.LRPosZ + Status.BodyPosZ + Status.BodyIKPosZ + Status.LRGaitPosZ, false);
  *Status.LRCoxaAngle = Status.IKCoxaAngle - LRCoxaOrrientation;
  *Status.LRFemurAngle = Status.IKFemurAngle;
  *Status.LRTibiaAngle = Status.IKTibiaAngle - TibiaOffset;
  
  LogAngles("beforeCorrection");
  CheckAngles();
}

void GaitSelect(int gaitType)
{  
    
     
    if (gaitType == 0) { //Ripple Gait 6 steps (one leg at a time alt. between sides)
        Status.LRGaitLegNr = 1;
        Status.RFGaitLegNr = 2;
        Status.LMGaitLegNr = 3;
        Status.RRGaitLegNr = 4;
        Status.LFGaitLegNr = 5;
        Status.RMGaitLegNr = 6;

        Status.StepsInGait = 6;
        Status.TLDivFactor = 4;
        Status.HalfLiftHeight = 0;
        Status.GaitType = 0;
     } else if (gaitType == 1) { // Tripod 4
        Status.LRGaitLegNr = 1;
        Status.RFGaitLegNr = 3;
        Status.LMGaitLegNr = 3;
        Status.RRGaitLegNr = 3;
        Status.LFGaitLegNr = 1;
        Status.RMGaitLegNr = 1;
        Status.StepsInGait = 4;
        Status.TLDivFactor = 2;
        Status.HalfLiftHeight = 0;
        Status.GaitType = 1;
      } else if (gaitType == 2) { //Wave 12 steps (one leg in the air down one side then the other)
        Status.LRGaitLegNr = 7;
        Status.RFGaitLegNr = 1;
        Status.LMGaitLegNr = 9;
        Status.RRGaitLegNr = 5;
        Status.LFGaitLegNr = 11;
        Status.RMGaitLegNr = 3;
        Status.StepsInGait = 12;
        Status.TLDivFactor = 10;
        Status.HalfLiftHeight = 0;
        Status.GaitType = 2;
      }
}

void Gait(int gaitLegNr, double* gaitPosX, double* gaitPosY, double* gaitPosZ, double* gaitRotY)
{
  // check if gait is in motion
  boolean gaitInMotion = ((abs(Status.TravelLengthX) > 2) || (abs(Status.TravelLengthZ) > 2) || (abs(Status.TravelRotationY) > 2));
  int nrLiftedPos = Status.StepsInGait - (Status.TLDivFactor + 1); 
 
  // Creates a smooth stopping cycle after movement
  if (((Status.GaitStep == gaitLegNr) && (abs(Status.TravelLengthX) == 0) &&  (abs(Status.TravelLengthZ) == 0) &&
                (abs(*gaitPosX) == 0) && (abs(*gaitPosZ) == 0) && (abs(*gaitRotY) == 0)) && (Status.CycleLegs > 0))
  {
    *gaitPosX = 0;
    *gaitPosY = Status.LegLiftHeight / 2.0;
    *gaitPosZ = 0;
    *gaitRotY = 0;

    // make sure legs are only cycled once
    Status.CycleLegs--;

  }
  // legs middle up position
  else if (((Status.GaitStep == gaitLegNr) && gaitInMotion && (nrLiftedPos == 1 || nrLiftedPos == 3)) || 
                (Status.GaitStep == gaitLegNr && !gaitInMotion && ((abs(*gaitPosX) > 2) || (abs(*gaitPosZ) > 2) || (abs(*gaitRotY) > 2))))
  {
     *gaitPosX = 0;
     *gaitPosY = Status.LegLiftHeight;
     *gaitPosZ = 0;
     *gaitRotY = 0;  
  }
  // Optional Half heigth Rear
  
  //Optional half heigth front
  
  //Leg front down position
  else if (((Status.GaitStep == gaitLegNr + nrLiftedPos) || (Status.GaitStep == gaitLegNr - (Status.StepsInGait - nrLiftedPos))) && (gaitPosY > 0))
  {
      *gaitPosX = Status.TravelLengthX / 2;
      *gaitPosY = 0;
      *gaitPosZ = Status.TravelLengthZ / 2;
      *gaitRotY = Status.TravelRotationY / 2;   
  }
  //Move body forward
  else
  {
      *gaitPosX -= Status.TravelLengthX / Status.TLDivFactor;
      *gaitPosY = 0;
      *gaitPosZ -= Status.TravelLengthZ / Status.TLDivFactor;
      *gaitRotY -= Status.TravelRotationY / Status.TLDivFactor;
  }
  
  if (Status.LastLeg)
  {
      Status.GaitStep++;
      if (Status.GaitStep > Status.StepsInGait)
      {
          Status.GaitStep = 1;
      }
  }
}

void BodyIK(double posX, double posZ, double bodyOffsetX, double bodyOffsetZ, double rotationY)
{
  double pitchY; //PitchY offset, needs to be added to the BodyPosY
  double rollY;  //RollY offset, needs to be added to the BodyPosY
  double totalX; //Total X distance between the center of the body and the feet
  double totalZ; //Total Z distance between the center of the body and the feet
  double distCenterBodyFeet;   //Total distance between the center of the body and the feet
  double angleCenterBodyFeetX; //Angle between the center of the body and the feet
  
  //Calculating totals from center of the body to the feet
  totalZ = bodyOffsetZ + posZ;
  totalX = bodyOffsetX + posX;
  
  //Distance between center body and feet
  distCenterBodyFeet = sqrt(((totalX * totalX) + (totalZ * totalZ)));

  //Angle X between center body and feet
  angleCenterBodyFeetX = (atan2(totalZ, totalX) * 180 / PI);

  //Calculate position corrections of feet X and Z for BodyRotation
  Status.BodyIKPosX = totalX - (cos((angleCenterBodyFeetX + (Status.BodyRotY + rotationY)) / 180 * PI) * distCenterBodyFeet);
  Status.BodyIKPosZ = totalZ - (sin((angleCenterBodyFeetX + (Status.BodyRotY + rotationY)) / 180 * PI) * distCenterBodyFeet);

  //Calculate position corrections for Y for Body Roll and Pitch
  rollY = (tan(((-Status.BodyRotZ) * PI) / 180.0) * (totalX));
  pitchY = (-tan(((Status.BodyRotX) * PI) / 180.0) * (totalZ));

  //Calculate total position correction in Y for Body Roll and Pitch
  Status.BodyIKPosY = rollY + pitchY;
}

void LegIK(double iKFeetPosX, double iKFeetPosY, double iKFeetPosZ, boolean isRightSide)
{
  double iKFeetPosXZ = 0; //Length between the coxa and feet
  double iKSW = 0;        //Length between shoulder and wrist
  double iKA1 = 0;        //Angle between SW line and the ground in rad
  double iKA2 = 0;        //?

  //reset all solutions options
  Status.IKSolution = false;
  Status.IKSolutionWarning = false;
  Status.IKSolutionError = false;

  //length between coxa and feet
  iKFeetPosXZ = sqrt(((iKFeetPosX * iKFeetPosX) + (iKFeetPosZ * iKFeetPosZ)));

  //IKSW - length between shoulder (femur servo) and tip of foot in the x/z plane
  iKSW = sqrt((((iKFeetPosXZ - CoxaLength) * (iKFeetPosXZ - CoxaLength)) +
          (iKFeetPosY * iKFeetPosY)));

  //ika1 - angle between femur line and the ground in rad    
  iKA1 = atan2(iKFeetPosY, iKFeetPosXZ - CoxaLength);

  //ika2 - law of cosines cos(c) = a^2 + b^2 - c^2 / 2ab
  iKA2 = acos((((FemurLength * FemurLength) - (TibiaLength * TibiaLength)) +
      (iKSW * iKSW)) / ((2 * FemurLength) * iKSW));

  //ikfemurangle
  Status.IKFemurAngle = -1 * ((iKA2 + iKA1) * 180) / 3.151592; // add negative angle
  
  //IKTibiaAngle
  Status.IKTibiaAngle =  (acos(((FemurLength * FemurLength) + (TibiaLength * TibiaLength) - (iKSW * iKSW)) / (2 * FemurLength * TibiaLength)) * 180.0 / PI);

  //IKCoxaAngle
  Status.IKCoxaAngle = -1 * ((atan2(iKFeetPosZ, iKFeetPosX) * 180.0) / PI);

  //if (isRightSide)
  //{
      //Status.IKFemurAngle *= -1;
      //Status.IKTibiaAngle *= -1;

      //if (iKFeetPosZ > 0)
      //Status.IKCoxaAngle *= -1;
  //}
  
  // set the solution quality

  if (iKSW < FemurLength + TibiaLength - 30)
  {
      Status.IKSolution = true;
  } else {
      if (iKSW < (FemurLength + TibiaLength))
      {
          Status.IKSolutionWarning = true;
      } else {
          Status.IKSolutionError = true;
      }
  }
}

void CheckAngles()
{
    *Status.RFCoxaAngle = round(min(max(*Status.RFCoxaAngle, servoAngleRanges[RFrontCoxaServo][0]), servoAngleRanges[RFrontCoxaServo][1]));
    *Status.RFFemurAngle = round(min(max(*Status.RFFemurAngle, servoAngleRanges[RFrontFemurServo][0]), servoAngleRanges[RFrontFemurServo][1]));
    *Status.RFTibiaAngle = round(min(max(*Status.RFTibiaAngle, servoAngleRanges[RFrontTibiaServo][0]), servoAngleRanges[RFrontTibiaServo][1]));

    *Status.RMCoxaAngle = round(min(max(*Status.RMCoxaAngle, servoAngleRanges[RCenterCoxaServo][0]), servoAngleRanges[RCenterCoxaServo][1]));
    *Status.RMFemurAngle = round(min(max(*Status.RMFemurAngle, servoAngleRanges[RCenterFemurServo][0]), servoAngleRanges[RCenterFemurServo][1]));
    *Status.RMTibiaAngle = round(min(max(*Status.RMTibiaAngle, servoAngleRanges[RCenterTibiaServo][0]), servoAngleRanges[RCenterTibiaServo][1]));

    *Status.RRCoxaAngle = round(min(max(*Status.RRCoxaAngle, servoAngleRanges[RRearCoxaServo][0]), servoAngleRanges[RRearCoxaServo][1]));
    *Status.RRFemurAngle = round(min(max(*Status.RRFemurAngle, servoAngleRanges[RRearFemurServo][0]), servoAngleRanges[RRearFemurServo][1]));
    *Status.RRTibiaAngle = round(min(max(*Status.RRTibiaAngle, servoAngleRanges[RRearTibiaServo][0]), servoAngleRanges[RRearTibiaServo][1]));

    *Status.LFCoxaAngle = round(min(max(*Status.LFCoxaAngle, servoAngleRanges[LFrontCoxaServo][0]), servoAngleRanges[LFrontCoxaServo][1]));
    *Status.LFFemurAngle = round(min(max(*Status.LFFemurAngle, servoAngleRanges[LFrontFemurServo][0]), servoAngleRanges[LFrontFemurServo][1]));
    *Status.LFTibiaAngle = round(min(max(*Status.LFTibiaAngle, servoAngleRanges[LFrontTibiaServo][0]), servoAngleRanges[LFrontTibiaServo][1]));

    *Status.LMCoxaAngle = round(min(max(*Status.LMCoxaAngle, servoAngleRanges[LCenterCoxaServo][0]), servoAngleRanges[LCenterCoxaServo][1]));
    *Status.LMFemurAngle = round(min(max(*Status.LMFemurAngle, servoAngleRanges[LCenterFemurServo][0]), servoAngleRanges[LCenterFemurServo][1]));
    *Status.LMTibiaAngle = round(min(max(*Status.LMTibiaAngle, servoAngleRanges[LCenterTibiaServo][0]), servoAngleRanges[LCenterTibiaServo][1]));

    *Status.LRCoxaAngle = round(min(max(*Status.LRCoxaAngle, servoAngleRanges[LFrontCoxaServo][0]), servoAngleRanges[LFrontCoxaServo][1]));
    *Status.LRFemurAngle = round(min(max(*Status.LRFemurAngle, servoAngleRanges[LFrontFemurServo][0]), servoAngleRanges[LFrontFemurServo][1]));
    *Status.LRTibiaAngle = round(min(max(*Status.LRTibiaAngle, servoAngleRanges[LFrontTibiaServo][0]), servoAngleRanges[LFrontTibiaServo][1]));
}

/*void morfeTo(double goal[18], double maxDeviation, int msecPerStep)
{
   double nextStep[18];
   bool difference = false;
   
   for (int i=0; i<18; i++)
   {
     nextStep[i] = goal[i] - servoAngles[i];
     Serial.print(nextStep[i]);
     Serial.print(" = ");
     Serial.print(goal[i]);
     Serial.print(" - ");
     Serial.println(servoAngles[i]);
     
     if (nextStep[i] == 0) continue;
     
     difference = true;
     if (nextStep[i] > 0) nextStep[i] = min(nextStep[i], maxDeviation);
     else if (nextStep[i] < 0) nextStep[i] = max(nextStep[i], -maxDeviation);
     
     nextStep[i] += servoAngles[i];
     
     Serial.print(" -> ");
     Serial.println(nextStep[i]);
   }
   
   if (!difference) return;
   
   setPositions(nextStep);
   delay(msecPerStep);
   morfeTo(goal, maxDeviation, msecPerStep);
}
*/

void setPositions(double angles[18])
{
  for (int i=0;i<18;i++)
  {
    if (servoPrevAngles[i] != angles[i])
    {
      Serial.print("Servo: ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(angles[i]);
      
      if (!isnan(angles[i]))
      {
        //servos[i].writeMicroseconds(calculateTiming(angles[i], i));
        servos[i].write((servoAngleDirection[i]*angles[i])-servoAngleOffset[i]);
        delayMicroseconds(10);
        servoPrevAngles[i] = angles[i];
      } else {
        Serial.println("NAN");
      }      
    }
  }
}

void setServoTimes(double times[18])
{
  for (int i=0;i<18;i++)
  {
      if (times[i] == 0)
        continue;
        
      Serial.print("Servo: ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(times[i]);
      servos[i].writeMicroseconds(times[i]);
      delayMicroseconds(10);
  }
}

/*
int calculateTiming(int angle, int servoNr)
{
  Serial.print("Angle: ");
  Serial.println(angle);
  int tempAngle = angle;
//  if (angle < servoAngleRanges[servoNr][0])
//    tempAngle = servoAngleRanges[servoNr][0];
//  if (angle > servoAngleRanges[servoNr][1])
//    tempAngle = servoAngleRanges[servoNr][1];
    
  int result = 1500 + (tempAngle * servoAngleFactors[servoNr]);
  Serial.print(" - result: ");
  Serial.println(result); 
  return result;
}
*/

// motion commands

void sleep()
{
 //double newAngles[] =  {0,-90,-70,0,-90,-70,0,-90,-70,0,-90,-70,0,-90,-70,0,-90,-70};
 //setPositions(newAngles);
 //delay(1000);
 detachServos();
}

void wake()
{
  attachServos(); 
  //double newAngles[] =  {0,-32.3,-49.5,0,-32.4,-49.5,0,-32.3,-49.5,0,-32.3,-49.5,0,-32.4,-49.5,0,-32.3,-49.5};
  //setPositions(newAngles);
}
/*
void startPosition()
{
 // R Rear (Coxa, Femur, Tibia, RCenter, RFront, LRear, L Center, L Front)
 //double newAngles[] =  {0, -85.43033235, -50.41572709, 0, -85.43033235, -50.41572709, 0, -85.43033235, -50.41572709, 0, -85.43033235, -50.41572709, 0, -85.43033235, -50.41572709, 0, -85.43033235, -50.41572709};
 //double newAngles[] =  {0, -30, 0, 0, -30, 0, 0, -30, 0, 0, -30, 0, 0, -30, 0, 0, -30, 0};
 double newAngles[] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
 setPositions(newAngles);
}

void goToStartPosition()
{
  double newAngles[18] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  morfeTo(newAngles, 5, 50);
}


void moveFromGround()
{
//  double newAngles[][18] = {
//    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, ////servoangles=0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
//    {0, 10, 0, 0, 10, 0, 0, 10, 0, 0, 10, 0, 0, 10, 0, 0, 10, 0}, //servoangles=0,10,0,0,10,0,0,10,0,0,10,0,0,10,0,0,10,0;
//    {0, 30, 0, 0, 30, 0, 0, 30, 0, 0, 30, 0, 0, 30, 0, 0, 30, 0}, //servoangles=0,30,0,0,30,0,0,30,0,0,30,0,0,30,0,0,30,0;
//    {0, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0}, //servoangles=0,50,0,0,50,0,0,50,0,0,50,0,0,50,0,0,50,0;
//    {0, 80, 0, 0, 80, 0, 0, 80, 0, 0, 80, 0, 0, 80, 0, 0, 80, 0}}; ////servoangles=0,80,0,0,80,0,0,80,0,0,80,0,0,80,0,0,80,0;
//    
//    for (int i=0;i<5;i++)
//    {
//      setPositions(newAngles[i]);
//      delay(200);
//    }
  double newAngles[18] =  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  morfeTo(newAngles, 2, 0);
}

void leftright()
{
  int angle = -40;
  double newAngle[18] = {-45, 0, 0, -45, 0, 0, -45, 0, 0, -45, 0, 0, -40, 0, 0, -40, 0, 0};
  //servoangles=-40,0,0,-40,0,0,-40,0,0,-40,0,0,-40,0,0,-40,0,0;
  //servotimes=2000,0,0,-2000,0,0,800,0,0,2500,0,0,-900,0,0,2100,0,0;
  //servoangles=-30,0,0,-30,0,0,-30,0,0,-30,0,0,-30,0,0,-30,0,0;
  //servoangles=0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
  //servoangles=40,0,0,40,0,0,40,0,0,40,0,0,40,0,0,40,0,0;
  
    
    while (angle <= 40)
    {
      for (int i=0;i<18;i=i+3)
        newAngle[i] = angle;
      
      setPositions(newAngle);
      angle += 10;
      delay(200);
    }
}
*/

void ripplegait6step()
{
  
   double newAngles[][18] = {
      {3.1,0.3,-4.4,-5.6,0,0.3,4.8,-28.3,-13.3,-4.8,1.1,11,11.1,0,1.1,0,0,0}, // RF
      {-40,80,0,   0,80,0,  0,20,0, -30,80,     0,20,0, 10,80,0}, // LC
      {-20,20,0, -10,80,0, 10,80,0, -20,80,0,  20,20,0, 0,80,0}, //RR
      {  0,20,0, -20,80,0, 20,80,0, -10,80,0,  10,80,0, 20,20,0}, //LF 
      {-10,80,0,   0,20,0, 30,80,0,   0,80,0,   0,80,0, 40,20,0}, //RC
      {-20,80,0,  20,20,0, 40,80,0, -20,20,0, -10,80,0, 30,80,0} // LR
    };
  
  while (moving)
  {
  for (int i=0;i<6;i++)
    {
      setPositions(newAngles[i]);
      delay(200);
    }
  }
}
