/*
* This program is written for an arduino Esplora and issues direct control commands to an iRobot Create 2 using
* serial communication. Code was originally developed as a control module for LEWIS, the mapping robot
* at Cincinnati Hills Christian Academy.
*
* Written by Adam Cool
* August 14, 2015
*
* Last Updated 5/11/2016
*
* iRobot Create 2 Open Interface Specifications may be found at http://www.irobotweb.com/~/media/MainSite/PDFs/About/STEM/Create/iRobot_Roomba_600_Open_Interface_Spec_0512.pdf?la=en
* iRobotCreate2 library thanks to DomAmato. Retrieved from: https://github.com/brinnLabs/Create2
*
* Wiring:
* pinout for Esplora is as follows from top to bottom on right headers (left headers are not used):
* +5V  - 
* +D14 - to LEWISTX
* +D15 - to LEWISRX
* +D16 - to LEWISBRC
* +D17 - to LCD14
* +D18 - to LCD13
* +D0  - to LCD12
* +D1  - to LCD11
* +5V  - 
* +GND - to LEWISGND
*
* pinout for Esplora across the top, left to right is as follows:
* 5V,      D11,     GND     __ 5V, D3,      GND     __ 5V,       ??, GND            __ 5V, ??, GND
* to LCD2, to LCD4, to LCD1,__ xx, to LCD6, to LCD5,__ to LCD15, xx, to LCD16/17/18 __ xx, xx, xx
*
* pinout for RGB backlit LCD (Adafruit product number 398) is as follows (left to right):
* 1: GND
* 2: 5V
* 3: Contrast pot
* 4: RS
* 5: GND (unused as RW pin)
* 6: EN
* 7-10: not used
* 11: DB4
* 12: DB5
* 13: DB6
* 14: DB7
* 15: 5V (backlight)
* 16: GND (backlight R signal)
* 17: GND (backlight G signal)
* 18: GND (backlight B signal)
*
* for pinout of iRobot Create 2 see Open Inteface Specifications document above
*
*
*
* Notes:
* - Esplora joystick range is -512 to 512, iRobot Create 2 valid drive speeds are -500 to 500, it is possible to slightly improve
*    precision of control by multiplying joystick values by 500/512. This is not a significant issue since values used are clamped at +/-500
*
* Questions:
* - How do I access sensor data once a stream is established?
*      - Do I need to append iRobotCreate2.cpp?
* - Can I integrate an LCD to the Esplora and utilize other ports for RX/TX?
* - What is current max speed? Is it battery dependent or does it utilize a feedback loop to maintain speed?
*
* To Do:
* - improve functionality of LED readout
* - add odometry zeroing function
* - integrate audio feedback from LEWIS
* - add button functionality
* - add readout screen to Esplora (need to switch comm pins to top ports)
* - increase max operating speed
* - read data from LEWIS (esp. odometry data X, Y, ?)
*    - transmit that data via TCP or other protocol to ROS
*
* Other possible improvements:
* - make Esplora control of LEWIS wireless
*/



#include <Esplora.h>
#include <iRobotCreate2.h>
#include <LiquidCrystal.h>

#define turnThresh 20		// dead band for joystick in x
#define fwdThresh 20		// dead band for joystick in y
#define turn_factor 1		// multiplier that adjusts the turning strength (originally 20 to offset threshold)
#define LCD_PIN_RS 11	
#define LCD_PIN_EN 3
#define LCD_PIN_D4 1
#define LCD_PIN_D5 0
#define LCD_PIN_D6 8
#define LCD_PIN_D7 7
#define ENCODER_JUMP_THRESHOLD 10000
#define ENCODER_MIN_VALUE -32768
#define ENCODER_MAX_VALUE 32767
#define WHEEL_DIAMETER 7.20 // in cm
#define WHEEL_BASE 23.5		// in cm
#define ENCODER_COUNTS_PER_REV	514.6 // Manual says 508.8, empirically, 514.6 leads to an error of less than 0.3%
#define ROTATION_CONVERSION_CONSTANT .10689378	 // approx. = WHEEL_DIAMETER / ENCODER_COUNTS_PER_REV) * 180) / (WHEEL_DIAMETER * PI) => .1126096 * .949242
#define WHEEL_REVS_TO_ROBOT_REVS .3063829787	// WHEEL_CIRCUMFERENCE / (PI*WHEEL_BASE) = WHEEL_RADIUS / WHEEL_BASE
#define DEFAULT_DISPLAY_MODE X_Y_TH
#define DEBUG_DISPLAY_MODE ENCODERS

bool isDebugOn = true;		// set to true to enable debug LCD display mode

int loopcount = 0;
byte LewisTX = 14;                                                        // Write pin for LEWIS serial communication (14)(12)
byte LewisRX = 15;                                                        // Read pin for LEWIS serial communication (15)(23)
byte LewisBRC = 16;                                                       // Pin for Baud Rate Change on LEWIS (16)(18)
byte instruction;                                                         // legacy variable used to transmit opcodes directly from serial monitor to LEWIS
byte dataPacket[4];
int timeGoal = 0;
double xCurrent, yCurrent;
double thCurrent, thPrevious = 0;
int testValue = 0;
int LEncoder, REncoder, LEncoderPrevious, REncoderPrevious, LEncoderInitial, REncoderInitial;
double throttle = 1;						// used to reduce the range of drive speed for better precision
double x_cm, y_cm, xPrevious_cm, yPrevious_cm;	// track x & y in centimeters 
String customMessage = "no msg";		// used for displaying messages to the LCD
int LEncoderRollovers = 0, REncoderRollovers = 0;		// used for tracking the number (and direction) of encoder rollovers in order to track total encoder accumulation
double distance_cm, theta;
bool wasLButtonPressed = false;

enum displayType { MESSAGE, TEST, ENCODERS, DISTANCE, ENCODER_DELTA_PER_SECOND, SPEED_AND_DIRECTION, BATTERY, X_Y_TH };
displayType displayState;
// when new values are added to displayType, left button press and displayOutput() need to be updated

// SoftwareSerial(int RX, int TX);
// SoftwareSerial LEWIS(LewisRX, LewisTX);                                // Establishes a software based serial communication port on the specified pins

// iRobotCreate2(bool useSoftSerial, byte rxPin, byte txPin, byte baudRateChangePin);
iRobotCreate2 LEWIS(true, LewisRX, LewisTX, LewisBRC);

// LiquidCrystal(int RSpin, int ENpin, int D4pin, int D5pin, int D6pin, int D7pin);
LiquidCrystal lcd(LCD_PIN_RS, LCD_PIN_EN, LCD_PIN_D4, LCD_PIN_D5, LCD_PIN_D6, LCD_PIN_D7);



void setup() {
	//	Serial.begin(57600);													  // used for debugging **MAY CAUSE PROBLEMS WITH LCD**

	lcd.begin(16, 2);

/*** initialize LEWIS baudrate and command mode ***/
	delay(2000);                                                              // must wait 2 seconds when robot is rebooted before changing baud rate,
																			  // include this line if arduino is powered directly by robot and loses
																			  // power when robot reboots
	initialize();
}

void initialize() {
	LEWIS.resetBaudRate();                                                    // When the robot is rebooted, serial communication occurs (by default)
																			  // at 115200 baud. Change this to 19200 baud for compatibility with Arduino
	delay(100);                                                               // must wait 100 ms after this command. See p.4 (iRC2 manual) for instructions

	LEWIS.start();
	delay(100);
	LEWIS.fullMode();                                                         // put robot into FULL control mode (Perhaps SAFE mode is better here?)

//	LEncoderInitial = LEWIS.getSensorData(leftEncoderCount);				  // blocking function to receive left encoder value
//	REncoderInitial = LEWIS.getSensorData(rightEncoderCount);				  // blocking function to receive right encoder value
/*** request data from LEWIS sensors ***/
	static byte requestIDs[2] = { leftEncoderCount, rightEncoderCount };
	static byte numOfRequests = 2;
	LEWIS.requestSensorDataList(numOfRequests, requestIDs);

	if (LEWIS.getSensorData(dataPacket, 4)) {
		LEncoderInitial = (dataPacket[0] << 8) + dataPacket[1];
		REncoderInitial = (dataPacket[2] << 8) + dataPacket[3];
	}
	else {
		lcd.clear();
		lcd.print("No encoder data");
	}

	LEncoderPrevious = LEncoderInitial;
	REncoderPrevious = REncoderInitial;
	thPrevious = 0;
	xCurrent = 0;
	yCurrent = 0;
	distance_cm = 0;
	theta = 0;

	setLewisMsg('r', 'e', 'd', 'y');                                          // indicate on LEWIS LCD, ready to receive commands
	customMessage = "LEWIS Connected";											  // indicate on control LCD, ready to receive commands
	displayState = MESSAGE;
}


void loop() {
	static int joyX, joyY, driveR, driveL;
	static int LEncoderDelta, REncoderDelta;
	static double xDelta, yDelta;
	static int distanceDelta_enc;
	static double distanceDelta_cm, thDelta;

//	use this if requesting streaming values instead of packets
//	static byte dataPacket[9];		// 9 bytes are: 19, 6(n-bytes), 43, LEncoder High Byte, LEncoder Low Byte, 44, REncoder High Byte, REncoder Low Byte, Checksum 

/*** request data from LEWIS sensors every 10 cycles ***/
	if (loopcount % 10 == 0) {
		static byte requestIDs[2] = { leftEncoderCount, rightEncoderCount };
		static byte numOfRequests = 2;
		LEWIS.requestSensorDataList(numOfRequests, requestIDs);

		if (LEWIS.getSensorData(dataPacket, 4)) {
			LEncoder = (dataPacket[0] << 8) + dataPacket[1];
			REncoder = (dataPacket[2] << 8) + dataPacket[3];
		}
		else {
			lcd.clear();
			lcd.print("No encoder data");
		}


		/*** Handle incoming encoder values ***/
		LEncoderDelta = LEncoder - LEncoderPrevious;
		REncoderDelta = REncoder - REncoderPrevious;
		if (LEncoderDelta > ENCODER_JUMP_THRESHOLD) {
			// LEncoder value rollover in reverse direction
			LEncoderDelta = (ENCODER_MIN_VALUE - LEncoderPrevious) + (LEncoder - ENCODER_MAX_VALUE);
			LEncoderRollovers--;
		}
		else if (LEncoderDelta < -ENCODER_JUMP_THRESHOLD) {
			// LEncoder value rollover in forward direction
			LEncoderDelta = (ENCODER_MAX_VALUE - LEncoderPrevious) + (LEncoder - ENCODER_MIN_VALUE);
			LEncoderRollovers++;
		}

		if (REncoderDelta > ENCODER_JUMP_THRESHOLD) {
			// REncoder value rollover in reverse direction
			REncoderDelta = (ENCODER_MIN_VALUE - REncoderPrevious) + (REncoder - ENCODER_MAX_VALUE);
			REncoderRollovers--;
		}
		else if (REncoderDelta < -ENCODER_JUMP_THRESHOLD) {
			// REncoder value rollover in forward direction
			REncoderDelta = (ENCODER_MAX_VALUE - REncoderPrevious) + (REncoder - ENCODER_MIN_VALUE);
			REncoderRollovers++;
		}

		/*** odometry calculations ***/
		distanceDelta_cm = (((double)LEncoderDelta + (double)REncoderDelta) * WHEEL_DIAMETER * PI / 2.0 / ENCODER_COUNTS_PER_REV);
		//	distanceDelta_enc = ((LEncoderDelta + REncoderDelta) / 2);

		//	what's wrong with this? thDelta = ((REncoderDelta - LEncoderDelta) * WHEEL_DIAMETER / ENCODER_COUNTS_PER_REV) * WHEEL_BASE;
		//	thDelta = (((REncoderDelta - LEncoderDelta) * WHEEL_DIAMETER / ENCODER_COUNTS_PER_REV) * 180) / (WHEEL_DIAMETER * PI);
		thDelta = ((double)REncoderDelta - (double)LEncoderDelta) * WHEEL_REVS_TO_ROBOT_REVS * 180.0 / ENCODER_COUNTS_PER_REV;  // in degrees

	//	thDelta = (REncoderDelta - LEncoderDelta) * ROTATION_CONVERSION_CONSTANT;
		thCurrent = thPrevious + thDelta;
		xDelta = distanceDelta_cm * sin(thCurrent * PI / 180.0);
		yDelta = distanceDelta_cm * cos(thCurrent * PI / 180.0);
		xCurrent = xPrevious_cm + xDelta;
		yCurrent = yPrevious_cm + yDelta;
		distance_cm += distanceDelta_cm;
		theta += thDelta;
	}


/*** LCD Display ***/
//	displayOutput(theta, distance_cm);
//	displayOutput(xPrevious_cm, xDelta);
	displayOutput();


/*** Esplora joystick controls ***/
	joyX = -Esplora.readJoystickX();										// joystick values are negated to make them intuitive
	joyY = -Esplora.readJoystickY();										// (positive values up and right)

	if (abs(joyY) < fwdThresh)
		joyY = 0;
	if (abs(joyX) < turnThresh)
		joyX = 0;

	throttle = (Esplora.readSlider() + 200) / (float)1223;                        // set throttle value with linear potentiometer range: 0.164 to 1.0

	driveR = (int)((joyY - joyX)*throttle);
	driveL = (int)((joyY + joyX)*throttle);

	LEWIS.driveWheels(driveR, driveL);                                         // This technique allows for faster turns and can be throttled																		   //  LEWIS.driveWheels(joyY-(joyX/2),joyY+(joyX/2));                         // This technique limits turn speed to half of maximum but allows for more nuanced turning

/*** Esplora button controls ***/
	if (Esplora.readButton(SWITCH_UP) == LOW) {
		initialize();
		//LEWIS.resetBaudRate();
		//delay(100);
		//LEWIS.start();
		//delay(100);
		//LEWIS.fullMode();
		//setLewisMsg('r', 'e', 'd', 'y');
	}

	/*** If right button is pressed, cycle through display modes ***/
	if (Esplora.readButton(SWITCH_LEFT) == LOW) {
		if (wasLButtonPressed == false) {
			wasLButtonPressed = true;
			switch (displayState) {
			case MESSAGE:
				if (isDebugOn == true)	displayState = DEBUG_DISPLAY_MODE;
				else if (isDebugOn == false)		displayState = DEFAULT_DISPLAY_MODE;
				break;
			case ENCODERS:
				displayState = DISTANCE;
				//			break;
			case DISTANCE:
				displayState = ENCODER_DELTA_PER_SECOND;
				//			break;
			case ENCODER_DELTA_PER_SECOND:
				displayState = SPEED_AND_DIRECTION;
				//			break;
			case SPEED_AND_DIRECTION:
				displayState = BATTERY;
				//			break;
			case BATTERY:
				displayState = X_Y_TH;
				break;
			case X_Y_TH:
				displayState = ENCODERS;
				break;
			case TEST:
				displayState = DEFAULT_DISPLAY_MODE;
				break;
			}
		}
	}
	else {
		wasLButtonPressed = false;
	}


	if (Esplora.readButton(SWITCH_RIGHT) == LOW) {
		xCurrent = 0.0;
		yCurrent = 0.0;
		thCurrent = 0.0;
		xPrevious_cm = 0.0;
		yPrevious_cm = 0.0;
		thPrevious = 0.0;
	}

/*** If bottom button is pressed, end communication with LEWIS ***/
	if (Esplora.readButton(SWITCH_DOWN) == LOW) {
		setLewisMsg('E', 'N', 'D', ' ');
		delay(1000);
		LEWIS.stop();
		customMessage = "LEWIS Disconnected";
		displayState = MESSAGE;
	}

/*** Update trailing values ***/
	LEncoderPrevious = LEncoder;
	REncoderPrevious = REncoder;
	xPrevious_cm = xCurrent;
	yPrevious_cm = yCurrent;
	thPrevious = thCurrent;

	loopcount++;
}



void displayOutput() {
	lcd.clear();  // clears LCD and repositions cursor to upper-left

	switch (displayState) {
	case MESSAGE:
		lcd.print(customMessage);
		if (customMessage.length() > 16) {
			lcd.setCursor(0, 1);
			lcd.print(customMessage.substring(16));
		}
		break;
	case ENCODERS:

		lcd.print("L: "); lcd.print(LEncoder);
		lcd.setCursor(0, 1);
		lcd.print("R: "); lcd.print(REncoder);

		break;
	case ENCODER_DELTA_PER_SECOND:
		break;
	case DISTANCE:
		break;
	case SPEED_AND_DIRECTION:
		break;
	case BATTERY:
		break;
	case X_Y_TH:
		/*** Display current X(ft), Y(ft), & Theta(deg) values on LCD ***/
		//		x_ft = x_current / 12;
		//		y_ft = y_current / 12;

		lcd.print("x: "); lcd.print(xCurrent, 1);
		lcd.setCursor(8, 0);  // repositions cursor to top-center
		lcd.print("y: "); lcd.print(yCurrent, 1);
		lcd.setCursor(0, 1);  // repositions cursor to bottom-left
		lcd.print((char)242);  // prints theta character
		lcd.print(": ");
		lcd.print(thCurrent, 1);
		break;
	case TEST:
//		lcd.print(a);
//		lcd.setCursor(0, 1);
//		lcd.print(b);
		break;
	}
}



void setLewisMsg(char cOne, char cTwo, char cThree, char cFour) {
	/* recieve 4 characters and post to LED display on LEWIS */
	LEWIS.setDigitLEDFromASCII(1, cOne);
	LEWIS.setDigitLEDFromASCII(2, cTwo);
	LEWIS.setDigitLEDFromASCII(3, cThree);
	LEWIS.setDigitLEDFromASCII(4, cFour);
}

void displayBattery() {
	/* display current battery percentage on LEWIS display */
	float charge = LEWIS.getSensorData(batterCharge);
	int pctCharge = charge / 655.35;
	LEWIS.setDigitLEDs(0, pctCharge / 100, (pctCharge % 100) / 10, pctCharge % 10);
}



/**** Deprecated Code ****/

/*
alternative method that allows for straight drive or rotation only
int velocity = 0;
int radius = 0;

if(abs(joyY) > fwdThresh){                                              //// fwd/rev motion intended
// joystick range: -512 to 512; drive velocity range: -500 to 500
velocity = (joyY - fwdThresh) * (500/(512 - fwdThresh));                // adjusts velocity to be a value from +/-1 to 500
radius = 32768;
}
} else if (joyX < -turnThresh){                                         //// turn CCW intended
radius = 1;
velocity = (abs(joyX) - turnThresh) * (500/(512 - turnThresh));         // adjusts turn velocity to be a value from 1 to 500
} else if (joyX > turnThresh){                                          //// turn CW intended
radius = -1;
velocity = (abs(joyX) - turnThresh) * (500/(512 - turnThresh));         // adjusts turn velocity to be a value from 1 to 500
}

LEWIS.drive(velocity, radius);                                            // Publish drive bytes to LEWIS
*/


/*
// Two's complement separator
byte vLowByte, vHighByte, rLowByte, rHighByte;
vHighByte = (unsigned int) velocity >> 8;
vLowByte = velocity % 256;
rHighByte = (unsigned int) radius >> 8;
rLowByte = radius % 256;
*/


// Display feedback from LEWIS in serial monitor; does not function b/c available is not a method of LEWIS (it works for direct softserial comm
//  if (LEWIS.available())
//    Serial.write(LEWIS.read(), INT);


//  if(softSerial.available()){
//    Serial.print("LEWIS output: ");
//    Serial.print(softSerial.read());


//int cStatus = LEWIS.getSensorData(21);


//Serial.print("Wall value: ");
//Serial.print(Wall);
//Serial.print("   charge: ");
//Serial.print(charge / 655.35);
//Serial.print("%  Charge status: ");
//Serial.print(cStatus);
//Serial.print("    delay1: ");
//Serial.print(midtime - pretime);
//Serial.print("   delay2: ");
//Serial.print(midtime2 - midtime);
//Serial.print("   delay3: ");
//Serial.println(posttime - midtime2);


/*
* Possible useful commands:
* - Seek Dock = 143
* - Power (down) = 133, changes to PASSIVE mode
* - Drive = 137 [Velocity high byte][Velocity low byte][Radius high byte][Radius low byte] **see p.12**
*      - Special cases: Straight = 32768, Rotate CW = -1, Rotate CCW = 1
* - Motors = 138, allows direct control of vaccuum fan
* - Digit LED Raw = 163, control the 4x LED digit segments (p. 16)
* - Digit N Bits = 165, emulate robot button push (p. 16)
* - Digit LEDs ASCII = 164
* - Song = 140
* - Sensors = 142, Query a sensor value
* - Query List = 143, Query multiple sensors (p.20)
*      Sensors:
*        - Bumper
*        - Wheel Drop
*        - Wall
*        - Cliff Left, Front Left, Front Right, Right
*        - Virtual Wall
*        - Wheel Overcurrents (jammed)
*        - Dirt Detect (0-255)
*        - IR Char Omni, Left, Right (p. 24)
*        - Buttons
*        - Distance (avg of 2 wheels in mm since last request) (p. 26)
*        - Angle (degrees) (p. 26)
*        - Charging state
*        - Voltage (in mV)
*        - Current (in mA)
*        - Temperature (of battery)
*        - Battery Charge
*        - Battery Capacity
*        - Left Encoder Count (Convert to mm: n*pi*72/508.8)(p. 30)
*        - Right Encoder Count
*        - Light Bump (w/ direction)
*
*/