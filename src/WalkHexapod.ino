void walk(int walkSpeedAndDirection) {
int intermission = 150; // delay between each part (in ms)
int horzDistance = walkSpeedAndDirection; // Distance to have each leg forward (actual distance of one
                       // step is 2*horzDistance because the leg moves horzDistance forward and backward)
int vertHeight = 200; // Total height of each step

int i;
for (i=5; i!=-1;i--) {
 //Vertical Servo1 Up:
 moveServo((servoNumber[i] + 6), (servoVertMid[i] + (servoVertDirection[i] * vertHeight)));
 //Horzizontal Servo1 to 2/4
 moveServo(servoNumber[i], HorzMid);

 //Vertical Servo2 Down:
 moveServo((servoNumber[i+1] + 6), servoVertMid[i+1]);
 //Horizontal Servo2 to 4/4 (front)
 moveServo(servoNumber[i+1], HorzMid + (servoHorzDirection[i+1] * horzDistance));

 //Horizontal Servo3 to 3/4
 moveServo(servoNumber[i+2], HorzMid + (servoHorzDirection[i+2] * (horzDistance/2)));

 //Horizontal Servo4 to 2/4 (midpoint)
 moveServo(servoNumber[i+3], HorzMid);

 //Horizontal Servo5 to 1/4
 moveServo(servoNumber[i+4], HorzMid - (servoHorzDirection[i+4] * (horzDistance/2)));

 //Vertical Servo6 Down:
 moveServo((servoNumber[i+5] + 6), servoVertMid[i+5]);
 //Horizontal Servo6 to 0/4 (back)
 moveServo(servoNumber[i+5], HorzMid - (servoHorzDirection[i+5] * horzDistance));

 //tell the servo controller to execute the commands and wait for the servos to finish
 Serial.println();
 delay(intermission);
}
}
