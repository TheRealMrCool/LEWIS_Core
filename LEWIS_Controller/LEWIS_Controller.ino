/*
* This code is meant to run on an Arduino Esplora with an LCD using a XBee Series 1 to
* communicate with an UNO, which, in turn, controls an iRobot Create 2. The was developed as the controller
* module for LEWIS, the mapping robot at Cincinnati Hills Christian Academy
*
* For wiring information, see LEWIS_Core
*
* TO DO:
* - decide behavior for init timeout
*
*/

#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <Esplora.h>

#define turnThresh 20		// dead band for joystick in x
#define fwdThresh 20		// dead band for joystick in y
#define turn_factor 1		// multiplier that adjusts the turning strength (originally 20 to offset threshold)
#define LCD_PIN_RS 0
#define LCD_PIN_EN 14
#define LCD_PIN_D4 15
#define LCD_PIN_D5 16
#define LCD_PIN_D6 7
#define LCD_PIN_D7 8
#define RX_PIN 11
#define TX_PIN 3
#define DEFAULT_DISPLAY_MODE X_Y_TH
#define HANDSHAKE_TIMEOUT 10000


int LEncoder, REncoder;					// stores current left & right encoder counts as reported by Core
double xCurrent, yCurrent, thCurrent;	// stores current x, y, & theta values as reported by Core
double throttle = 1;					// used to reduce the range of drive speed for better precision
String customMessage = "no msg";		// used for displaying messages to the LCD
unsigned long startClock;						// used to compare the time since an action was taken
byte readByte;
byte handShaken = 0, initialized = 0;

bool wasLButtonPressed = false;

enum displayType { MESSAGE, TEST, ENCODERS, DISTANCE, ENCODER_DELTA_PER_SECOND, SPEED_AND_DIRECTION, BATTERY, X_Y_TH };
displayType displayState;
// when new values are added to displayType, left button press and displayOutput() need to be updated


enum LEWISCommand {
	// LEWISCommand contains the list of codes sent over serial to LEWIS_Core
	INIT = 1,	// instruct Core to begin initialization procedure (core should return 100 if successful)
	DRIVE = 2,	// send left and right motor drive commands (+2 bytes each)
	RESET = 3,	// instruct Core to zero x, y, & theta values (set current position as home)
	END = 4,	// instruct Core to cease communication with LEWIS
	SETMSG = 5,	// set LEWIS LED display message (+4 bytes - 1 char each)
	SHAKEHAND = 6	// acknowledge connection to Core
};

enum LEWISReport {
	// LEWISReport contains the list of status codes sent over serial to LEWIS_Controller
	INITIALIZED = 100,	// LEWIS successfully initialized
	SENDALL = 101,		// Sending all odometry and status values for display (+10 bytes - left and right encoders, x, y, & theta
	HANDSHAKE = 111,	// Acknowledge connection with Controller 
	ERROR = 200			// Report an error (+1 byte - error code)
};

enum LEWISError {
	// LEWISError contains the list of error codes that follow a reported error
	INIT_TIMEOUT = 1,
	SHAKE_TIMEOUT = 2,
	UNINITIALIZED = 3
};

LiquidCrystal lcd(LCD_PIN_RS, LCD_PIN_EN, LCD_PIN_D4, LCD_PIN_D5, LCD_PIN_D6, LCD_PIN_D7);
SoftwareSerial SerialToCore(RX_PIN, TX_PIN); // establishes serial communication with XBee



void setup() {
	SerialToCore.begin(19200);
	lcd.begin(16, 2);
	if (initiateHandshake() == 1) {
		handShaken = 1;
		initialized = initialize();
	}
	else {
		handleError(SHAKE_TIMEOUT);
		handShaken = 0;
	}
}


void loop() {
	static int joyX, joyY, driveR, driveL;
	static byte driveRHigh, driveRLow, driveLHigh, driveLLow;	// stores the byte separation of drive commands

	if (handShaken == 0) {
		while (!(Esplora.readButton(SWITCH_UP) == LOW)) {
			// hang until the top button is pressed
			lcd.clear();
			lcd.println("no handshake");
			lcd.setCursor(0, 1);
			lcd.println("TRY AGAIN?");
		}
		handShaken = initiateHandshake();
	}

	if (initialized == 1) {
		// ** check for new data packet and store in the appropriate variables

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

		driveRHigh = driveR >> 8;
		driveRLow = driveR;
		driveLHigh = driveL >> 8;
		driveLLow = driveL;

		// report drive commands to Core
		SerialToCore.write(DRIVE);
		SerialToCore.write(driveRHigh);
		SerialToCore.write(driveRLow);
		SerialToCore.write(driveLHigh);
		SerialToCore.write(driveLLow);

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
					displayState = DEFAULT_DISPLAY_MODE;
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
			// ** send command to zero x, y, & th
		}

		/*** If bottom button is pressed, end communication with LEWIS ***/
		if (Esplora.readButton(SWITCH_DOWN) == LOW) {
			// ** send command to disconnect from LEWIS
			customMessage = "LEWIS Disconnected";
			displayState = MESSAGE;
		}
	}
	else if (handShaken == 1) {
		// comm with core exists but core is not initialized
		handleError(UNINITIALIZED);
	}

	/*** LCD Display ***/
	displayOutput();
}


int initialize() {
// sends op code to core to initialize. Waits for a response up to INIT_TIMEOUT. If response is 100 (LEWIS INITIALIZED),
// then print message to LCD, if response is 200 (error), handle error
	
	SerialToCore.write(INIT);
	while (!SerialToCore.available())
	{
	// wait until Core sends status
	}

	byte readByte = SerialToCore.read();
	if (readByte == 100) {
		// LEWIS is properly initialized
// ** notify user that LEWIS is connected
		lcd.print("Initialized");
		return 1;
	}
	else if (readByte == 200) {
		handleError(SerialToCore.read());
		return 0;
	}
	else {
		// this case should never happen or LEWIS communication is not functioning properly
// ** notify user that LEWIS comm is BAD
		return 0;
	}
}


int initiateHandshake() {
	SerialToCore.write(SHAKEHAND);
	startClock = millis();
	while ((millis() - startClock) < HANDSHAKE_TIMEOUT) {
		if (SerialToCore.available()) {
			readByte = SerialToCore.read();
			// message received
			if (readByte == HANDSHAKE) return 1;	// return 1 if handshake successful
		}
	}
	// handshake has timed out
	return 0;

}


void handleError(byte errorCode) {
// handles any error codes sent from Core
	switch (errorCode) {
	case INIT_TIMEOUT:
// ** display message that initialization has timed out...repeat init attempt?
		customMessage = "init. timeout";
		displayState = MESSAGE;
		break;
	case SHAKE_TIMEOUT:
// ** display message that handshake has timed out...???
		lcd.clear();
		customMessage = "handsh. timeout";
		lcd.println(customMessage);
		displayState = MESSAGE;
		break;
	case UNINITIALIZED:
// ** reinitialize???
		break;
	default:

		break;
	}
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