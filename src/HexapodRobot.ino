#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver rPwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver lPwm = Adafruit_PWMServoDriver(0x41);

int servo = -1;
int freq = -1;
int side = 0;

#define SERV_QTY 9
#define PROP_QTY 3

uint16_t rServos[SERV_QTY][PROP_QTY];
uint16_t lServos[SERV_QTY][PROP_QTY];

void initServoParameters() {
	//Set max, min and initial values for all servos...
	//Leg 0
		rServos[0][0] = 260; //min value
		rServos[0][1] = 590; //max value
		rServos[0][2] = 300; //initial value

		rServos[1][0] = 90; //min value 90
		rServos[1][1] = 0;  //max value
		rServos[1][2] = 0;  //initial value

		rServos[2][0] = 0;  //min value
		rServos[2][1] = 0;  //max value
		rServos[2][2] = 0;  //initial value

	//Leg 1
		rServos[3][0] = 0;  //min value
		rServos[3][1] = 0;  //max value
		rServos[3][2] = 0;  //initial value

		rServos[4][0] = 0;  //min value
		rServos[4][1] = 0;  //max value
		rServos[4][2] = 0;  //initial value

		rServos[5][0] = 0;  //min value
		rServos[5][1] = 0;  //max value
		rServos[5][2] = 0;  //initial value

	//Leg 2
		rServos[6][0] = 0;  //min value
		rServos[6][1] = 0;  //max value
		rServos[6][2] = 0;  //initial value

		rServos[7][0] = 0;  //min value
		rServos[7][1] = 0;  //max value
		rServos[7][2] = 0;  //initial value

		rServos[8][0] = 0;  //min value
		rServos[8][1] = 0;  //max value
		rServos[8][2] = 0;  //initial value

	//Leg 3
		lServos[0][0] = 0;  //min value
		lServos[0][1] = 0;  //max value
		lServos[0][2] = 0;  //initial value

		lServos[1][0] = 0;  //min value 90
		lServos[1][1] = 0;  //max value
		lServos[1][2] = 0;  //initial value

		lServos[2][0] = 0;  //min value
		lServos[2][1] = 0;  //max value
		lServos[2][2] = 0;  //initial value

	//Leg 4
		lServos[3][0] = 0;  //min value
		lServos[3][1] = 0;  //max value
		lServos[3][2] = 0;  //initial value

		lServos[4][0] = 0;  //min value
		lServos[4][1] = 0;  //max value
		lServos[4][2] = 0;  //initial value

		lServos[5][0] = 0;  //min value
		lServos[5][1] = 0;  //max value
		lServos[5][2] = 0;  //initial value

	//Leg 5
		lServos[6][0] = 0;  //min value
		lServos[6][1] = 0;  //max value
		lServos[6][2] = 0;  //initial value

		lServos[7][0] = 0;  //min value
		lServos[7][1] = 0;  //max value
		lServos[7][2] = 0;  //initial value

		lServos[8][0] = 0;  //min value
		lServos[8][1] = 0;  //max value
		lServos[8][2] = 0;  //initial value
}

void setup() {

	initServoParameters();

	servo = -1;
	freq = -1;
	side = 0;

	Serial.begin(19200);
	Serial.write("Power On");

	rPwm.begin();
	rPwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

	lPwm.begin();
	lPwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

	InitialPosition();

	yield();
}



void InitialPosition() {
	for (uint8_t i = 0; i < SERV_QTY; i++)
	{
		rPwm.setPWM(i, 0, rServos[i][3]);
		lPwm.setPWM(i, 0, lServos[i][3]);
	}
	Serial.println(" -> Reseted to Initial Position! ");
}

/**
 * Function that reads serial strings and returns it
 **/
String ReadSerialString() {
	String content = "";
	char letter;

	// While receiving something from serial...
	while (Serial.available() > 0) {
		// Read byte from serial
		letter = Serial.read();
		// Ignore break row char
		if (letter != '\n') {
			// Concact values
			content.concat(letter);
		}
		// wait for serial buffer read next letters
		delay(10);
	}
	return content;
}


void loop() {
	// If receives something from serial...
	if (Serial.available() > 0) {
		// read string received
		String received = ReadSerialString();
		received.toUpperCase();

		if (received.substring(0, 1) == "I") {
			InitialPosition();
		}

		if (received.substring(0, 1) == "S") {
			servo = received.substring(1).toInt();
			Serial.print(" -> Servo: ");
			Serial.println(servo);
		}

		if (received.substring(0, 1) == "F") {
			freq = received.substring(1).toInt();
			Serial.print(" -> Frequency: ");
			Serial.println(freq);
		}

		if (received.substring(0, 1) == "L" || received.substring(0, 1) == "E") {
			Serial.println(" -> Left Side ");
			side = 0;
		}

		if (received.substring(0, 1) == "R" || received.substring(0, 1) == "D") {
			Serial.println(" -> Right Side ");
			side = 1;
		}

		delay(250);
	}

	if (servo > -1 && freq > -1)
	{
		if (side == 0) {
			lPwm.setPWM(servo, 0, freq);
			Serial.print(" -> Moved: Left Servo ");
			Serial.print(servo);
			Serial.print(" to Frequency ");
			Serial.println(freq);
		}
		else {
			rPwm.setPWM(servo, 0, freq);
			Serial.print(" -> Moved: Right Servo ");
			Serial.print(servo);
			Serial.print(" to Frequency ");
			Serial.println(freq);
		}
		freq = -1;
		delay(2000);
	}
}