/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

const double femur = 14.0; // in cm
const double tibia = 20.0;
const double coxa = 3.0; 
double Z_OFFSET = 0.0;

const int L_MIN = 15;
const int L_MAX = 12;


const int servoN10 = 0;
const int servoN10Min = 105;
const int servoN10Max = 480;
const int sheildN10 = 0;
const int initThetaN10 = 90;

const int servoN11 = 1;
const int servoN11Min = 85;
const int servoN11Max = 440;
const int sheildN11 = 0;
const int initThetaN11 = 0;
const int thetaOffsetN11 = 15;

const int servoN12 = 2;
const int servoN12Min = 95;
const int servoN12Max = 450;
const int sheildN12 = 0;
const int initThetaN12 = 0;
const int thetaN12Skew = 45.0; // servo's 0 pos is set at 45 degree angle

double getAlpha(double h, double f, double servoLift) {
  double L1 = sqrt(pow(h,2)+ pow(f,2));
  double z = Z_OFFSET + servoLift;
  double L = sqrt(pow(z,2) + pow(L1-coxa,2)); // L is radial dist from servo 2
                                                              // L1 is horizontal distance from center
  double alpha1 = acos(z/L);
  double alpha2 = acos( (pow(femur,2) + pow(L,2) - pow(tibia,2)) / (2.0*femur*L) );

  double alpha = 180*(alpha1 + alpha2) / M_PI;
  if (alpha == NAN)  {
    return 0;
  }
  return alpha;
}

double getBeta(double h, double f, double servoLift) {
  double L1 = sqrt(pow(h,2)+ pow(f,2));
  double z = Z_OFFSET + servoLift;
  double L = sqrt(pow(z,2) + pow(L1-coxa,2)); // L is radial dist from servo 2
                                                              // L1 is horizontal distance from center  
  if (L > femur + tibia) {
    return 0;
  }
  Serial.print("in beta, l:"); Serial.println(L);
  double beta = (180.0/M_PI)*acos( ( pow(femur,2) + pow(tibia, 2) - pow(L,2) )/(2.0*femur*tibia) );
  return beta;
}

double getGamma(double h, double f) {
  double L1 = sqrt(pow(h,2)+ pow(f,2));

  double gamma = 180.0*(acos( h / L1)) / M_PI;
  Serial.print("gamma val:"); Serial.println(gamma);
  if (gamma == NAN)  {
    return 90;
  }

  if(gamma > 180 || gamma < 0) {
    return 90;
  }

  if(f < 0) {
    gamma = 0 - gamma;
  }
  return gamma;
}


void setServoPosition(int servoNum, int theta) {

  if (theta < 0 || theta > 180) {
    Serial.println("invalid theta:" + theta);
    return;
  }

  int shieldNum = 0;
  int pos0 = -1;
  int pos180 = -1;
  int thetaOffset = 0.0;
  switch(servoNum) {
    case servoN10:
      pos0 = servoN10Min;
      pos180 = servoN10Max;
      shieldNum = 0;
      break;  
    case servoN11:
      pos0 = servoN11Min;
      pos180 = servoN11Max;
      thetaOffset = 15;
      shieldNum = 0;
      break;  
    case servoN12:
      pos0 = servoN12Min;
      pos180 = servoN12Max;
      shieldNum = 0;
      break;  

    default:
      Serial.println("invalid servo:" + servoNum );
      return;                                
  }

  if (pos0 < 0 || pos180 < 0) {
    Serial.println("invalid servo:" + servoNum);
    return;
  }

  if (theta < thetaOffset) {
    theta = thetaOffset;
  }
  double pulseLength = map(theta, 0, 180, pos0, pos180);
  if(shieldNum == 0) {
    pwm.setPWM(servoNum, 0, pulseLength);
  } else {
    pwm2.setPWM(servoNum, 0, pulseLength);
  }
  
}

void setup() {
  
  
  Serial.begin(9600);
  Serial.println("16 channel, dual shield, Servo test!");

  pwm.begin();
  pwm.setPWMFreq(50); 
  

  yield();
}

void setArmPosition(double h, double f, double servoLift) {
  double gamma = getGamma(h, f);
  Serial.print("gamma:"); Serial.println(gamma);
  Serial.print("90 - gamma:"); Serial.println(90 - gamma);
  
  double alpha = getAlpha(h, f, servoLift);
  Serial.print("180 - alpha:"); Serial.println(180 - alpha);

  double beta = getBeta(h, f, servoLift);
  Serial.print("beta:"); Serial.println(beta);

  setServoPosition(servoN10, 90 - gamma);
  setServoPosition(servoN11, 180 - alpha);
  setServoPosition(servoN12, beta - 45);  
}

void testMoveStraight() {
  double h = 20;  //horizontal distance from center
  //double f = -10; // forward or backward distance
  int servoLift = 8; // height of center above ground

  for(double f = 20.0; f >= -20; f = f-1) {
    setArmPosition(h, f, servoLift);
    delay(100);
  }
  
  for(double f = -20.0; f <= 20; f = f+1) {
    setArmPosition(h, f, servoLift);
    delay(100);
  }  
}

void testMoveSideways() {
  double f = 0;  //horizontal distance from center
  int servoLift = 8; // height of center above ground

  for(double h = 30.0; h >= 15; h = h-1) {
    setArmPosition(h, f, servoLift);
    delay(100);
  }
  
  for(double h = 15.0; h <= 30; h = h+1) {
    setArmPosition(h, f, servoLift);
    delay(100);
  }  
}


void testMoveUpwards() {
  double f = 0;  //horizontal distance from center
  double h = 25;
  int servoLift = 8; // height of center above ground

  for(double servoLift = 8.0; servoLift >= -20; servoLift = servoLift-1) {
    setArmPosition(h, f, servoLift);
    delay(1000);
  }
  
  for(double servoLift = -20.0; servoLift <= 8; servoLift = servoLift+1) {
    setArmPosition(h, f, servoLift);
    delay(1000);
  }  
}

void loop() {

    testMoveStraight();
    testMoveSideways();
    //testMoveUpwards();

  

  delay(4000);

}