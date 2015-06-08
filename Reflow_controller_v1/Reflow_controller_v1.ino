/*******************************************************************************
* Title: Reflow Oven Controller
* Version: 1.20
* Date: 26-11-2012
* Company: Rocket Scream Electronics
* Author: Lim Phang Moh
* Website: www.rocketscream.com
* https://github.com/rocketscream/Reflow-Oven-Controller
* 
* Brief
* =====
* This is an example firmware for our Arduino compatible reflow oven controller. 
* The reflow curve used in this firmware is meant for lead-free profile 
* (it's even easier for leaded process!). You'll need to use the MAX31855 
* library for Arduino if you are having a shield of v1.60 & above which can be 
* downloaded from our GitHub repository. Please check our wiki 
* (www.rocketscream.com/wiki) for more information on using this piece of code 
* together with the reflow oven controller shield. 
*
* Temperature (Degree Celcius)                 Magic Happens Here!
* 245-|                                               x  x  
*     |                                            x        x
*     |                                         x              x
*     |                                      x                    x
* 200-|                                   x                          x
*     |                              x    |                          |   x   
*     |                         x         |                          |       x
*     |                    x              |                          |
* 150-|               x                   |                          |
*     |             x |                   |                          |
*     |           x   |                   |                          | 
*     |         x     |                   |                          | 
*     |       x       |                   |                          | 
*     |     x         |                   |                          |
*     |   x           |                   |                          |
* 30 -| x             |                   |                          |
*     |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
*     | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
*  0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
*                                                                Time (Seconds)
*
* This firmware owed very much on the works of other talented individuals as
* follows:
* ==========================================
* Brett Beauregard (www.brettbeauregard.com)
* ==========================================
* Author of Arduino PID library. On top of providing industry standard PID 
* implementation, he gave a lot of help in making this reflow oven controller 
* possible using his awesome library.
*
* ==========================================
* Limor Fried of Adafruit (www.adafruit.com)
* ==========================================
* Author of Arduino MAX6675 library. Adafruit has been the source of tonnes of
* tutorials, examples, and libraries for everyone to learn.
*
* Disclaimer
* ==========
* Dealing with high voltage is a very dangerous act! Please make sure you know
* what you are dealing with and have proper knowledge before hand. Your use of 
* any information or materials on this reflow oven controller is entirely at 
* your own risk, for which we shall not be liable. 
*
* Licences
* ========
* This reflow oven controller hardware and firmware are released under the 
* Creative Commons Share Alike v3.0 license
* http://creativecommons.org/licenses/by-sa/3.0/ 
* You are free to take this piece of code, use it and modify it. 
* All we ask is attribution including the supporting libraries used in this 
* firmware. 
*
* Required Libraries
* ==================
* - Arduino PID Library: 
*   >> https://github.com/br3ttb/Arduino-PID-Library
* - MAX31855 Library (for board v1.60 & above): 
*   >> https://github.com/rocketscream/MAX31855
* - MAX6675 Library (for board v1.50 & below):
*   >> https://github.com/adafruit/MAX6675-library
*
* Revision  Description
* ========  ===========
* 1.20			Adds supports for v1.60 (and above) of Reflow Oven Controller 
*           Shield:
*					  - Uses MAX31855KASA+ chip and pin reassign (allowing A4 & A5 (I2C)
*             to be used for user application).
*					  - Uses analog based switch (allowing D2 & D3 to be used for user 
*						  application).	
*						Adds waiting state when temperature too hot to start reflow process.
*						Corrected thermocouple disconnect error interpretation (MAX6675).
* 1.10      Arduino IDE 1.0 compatible.
* 1.00      Initial public release.
*******************************************************************************/

// ***** INCLUDES *****
#include <SoftwareSerial.h>
#include <SPI.h>

// #include <MAX31855.h> // https://github.com/rocketscream/MAX31855
#include <Adafruit_MAX31855.h> // https://github.com/adafruit/Adafruit-MAX31855-library

#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library/

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
	REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;

typedef	enum SWITCH
{
	SWITCH_NONE,
	SWITCH_1,	
	SWITCH_2
}	switch_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

// ***** CONSTANTS *****
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 130	// lead-free ~150C, leaded ~130C
#define TEMPERATURE_SOAK_MAX 180	// lead-free ~200C, leaded ~180C
#define TEMPERATURE_REFLOW_MAX 225 // lead-free ~250C, leaded ~225C
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5
#define SOAK_MICRO_PERIOD 9000
#define DEBOUNCE_PERIOD_MIN 50

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100		// default 100
#define PID_KI_PREHEAT 0.025 	// default 0.025
#define PID_KD_PREHEAT 20 		// default 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300 		// default 300
#define PID_KI_SOAK 0.05 		// default 0.05
#define PID_KD_SOAK 250 		// default 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300 		// default 300
#define PID_KI_REFLOW 0.05  	//default 0.05
#define PID_KD_REFLOW 350		// default 350
#define PID_SAMPLE_TIME 1000	//default 1000



// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
  "Press start butt",
  "Pre-heat",
  "Soak",
  "Reflow",
  "Cool",
  "Complete",
  "Wait,hot",
  "Error"
};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {
  140,146,146,140,128,128,128,128};

// ***** PIN ASSIGNMENT *****
// Pin assignments altered to go with Luke's 
// Reflow_board_v1 designed in Eagle 2015-06-06

int ssrPin = 5; // Arduino pin D5
int thermocoupleSOPin = 12; // MISO
int thermocoupleCSPin = 10; // Chip Select
int thermocoupleCLKPin = 13; // SCK pin for SPI
int ledRedPin = 6; // LED on Arduino pin D6
int switch1Pin = 2; // Button1 on Arduino pin D2 (aka INT0)


// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
unsigned long buzzerPeriod;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
// Seconds timer
int timerSeconds;

int roundSet; // variable for rounded-off setpoint temperature

// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Settings for SoftwareSerial to talk to Sparkfun SerialLCD 16x2 connected
// to Arduino pin D3.
SoftwareSerial LCD(4,3); // pin3 = TX to LCD RX pin, pin4 = RX (unused)


// Rocketscream Max31855 library
// MAX31855 thermocouple(thermocoupleSOPin, thermocoupleCSPin, thermocoupleCLKPin);
// Adafruit Max31855 library
Adafruit_MAX31855 thermocouple(thermocoupleCLKPin, thermocoupleCSPin, thermocoupleSOPin);
											
void setup()
{
  // SSR pin initialization to ensure reflow oven is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  // Serial communication at 57600 bps
  Serial.begin(57600);
  Serial.println(F("Booting"));
  // LED pins initialization and turn on upon start-up (active high)
  pinMode(ledRedPin, OUTPUT);
  digitalWrite(ledRedPin, LOW); // turn LED off if SSR is off
  // Button pin initialization
  pinMode(switch1Pin, INPUT_PULLUP);


  LCD.begin(9600); // setup software serial to talk to Sparkfun serial LCD
  delay(1000); // let LCD boot up
  clearScreen();
  backlight(157); // turn backlight on full
  delay(10);
  LCD.print(F("Reflow"));
  selectLineTwo();
  LCD.print(F("Controller"));
  delay(1000);
  clearScreen();

  // Set window size
  windowSize = 2000;
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();
}

void loop()
{
  // Current time
  unsigned long now;

  // Time to read thermocouple?
  if (millis() > nextRead)
  {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature
	// input = thermocouple.readThermocouple(CELSIUS); // rocketscream library
	input = thermocouple.readCelsius(); // Adafruit library

		
    // If thermocouple problem detected
	if(isnan(input)) {
    //  Illegal operation
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
    }
  }

  if (millis() > nextCheck)
  {
    // Check input in the next seconds
    nextCheck += 1000;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Toggle red LED on to show relay is powered on
      digitalWrite(ledRedPin, HIGH);
      // Increase seconds timer for reflow curve analysis
      timerSeconds++;
      // Send temperature and time stamp to serial 
      Serial.print(timerSeconds);
      Serial.print(", ");
      Serial.print(setpoint); // Target temperature
      Serial.print(", ");
      Serial.print(input); // Temperature value from MAX31855
      Serial.print(", ");
      Serial.println(output); // output from PID calculation
	  
    }
    else
    {
      // Turn off red LED
      digitalWrite(ledRedPin, LOW);
    }

    // Clear LCD
    clearScreen();
	selectLineOne();
	// Print current system state
	if (reflowState == REFLOW_STATE_IDLE) {
		// The idle state message is longer than the others,
		// so don't print setpoint temperature
		LCD.print(lcdMessagesReflowStatus[reflowState]);
		
	} else {
		// Print state and setpoint temperature
		LCD.print(lcdMessagesReflowStatus[reflowState]);
		LCD.print(F(" Set:"));
		roundSet = (int)setpoint; // round off setpoint to nearest degree
		LCD.print(roundSet);
	}
    selectLineTwo();
    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {
      // No thermocouple wire connected
      LCD.print("TC Error!");
    }
    else
    {
	  LCD.print(F("Temp: "));
      // Print current temperature
      LCD.print(input);
      LCD.print("C");
    }
  }

  // Reflow oven controller state machine
  switch (reflowState)
  {
  case REFLOW_STATE_IDLE:
		// If oven temperature is still above room temperature
		if (input >= TEMPERATURE_ROOM)
		{
			reflowState = REFLOW_STATE_TOO_HOT;
		}
		else
		{
			// If switch is pressed to start reflow process
			if (switchStatus == SWITCH_1)
			{
        // Send header for CSV file
        Serial.println(F("Time, SetpointTemp, InputTemp, PIDOutput"));
        // Initialize seconds timer for serial debug information
        timerSeconds = 0;
        // Initialize PID control window starting time
        windowStartTime = millis();
        // Ramp up to minimum soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN;
        // Tell the PID to range between 0 and the full window size
        reflowOvenPID.SetOutputLimits(0, windowSize);
        reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
        // Turn the PID on
        reflowOvenPID.SetMode(AUTOMATIC);
        // Proceed to preheat stage
        reflowState = REFLOW_STATE_PREHEAT;
      }
    }
    break;

  case REFLOW_STATE_PREHEAT:
    reflowStatus = REFLOW_STATUS_ON;
    // If minimum soak temperature is achieve       
    if (input >= TEMPERATURE_SOAK_MIN)
    {
      // Chop soaking period into smaller sub-period
      timerSoak = millis() + SOAK_MICRO_PERIOD;
      // Set less agressive PID parameters for soaking ramp
      reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
      // Ramp up to first section of soaking temperature
      setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;   
      // Proceed to soaking state
      reflowState = REFLOW_STATE_SOAK; 
    }
    break;

  case REFLOW_STATE_SOAK:     
    // If micro soak temperature is achieved       
    if (millis() > timerSoak)
    {
      timerSoak = millis() + SOAK_MICRO_PERIOD;
      // Increment micro setpoint
      setpoint += SOAK_TEMPERATURE_STEP;
      if (setpoint > TEMPERATURE_SOAK_MAX)
      {
        // Set agressive PID parameters for reflow ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_REFLOW_MAX;   
        // Proceed to reflowing state
        reflowState = REFLOW_STATE_REFLOW; 
      }
    }
    break; 

  case REFLOW_STATE_REFLOW:
    // We need to avoid hovering at peak temperature for too long
    // Crude method that works like a charm and safe for the components
    if (input >= (TEMPERATURE_REFLOW_MAX - 5))
    {
      // Set PID parameters for cooling ramp
      reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
      // Ramp down to minimum cooling temperature
      setpoint = TEMPERATURE_COOL_MIN;   
      // Proceed to cooling state
      reflowState = REFLOW_STATE_COOL; 
    }
    break;   

  case REFLOW_STATE_COOL:
    // If minimum cool temperature is achieve       
    if (input <= TEMPERATURE_COOL_MIN)
    {
		clearScreen();
		selectLineOne();
		LCD.print(F("Cooldown done"));
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;                
      // Proceed to reflow Completion state
      reflowState = REFLOW_STATE_COMPLETE; 
    }         
    break;    

  case REFLOW_STATE_COMPLETE:
    {
	  clearScreen();
	  selectLineOne();
	  LCD.print(F("Reflow complete"));
	  digitalWrite(ledRedPin, LOW);
      reflowState = REFLOW_STATE_IDLE; 
    }
    break;
	
	case REFLOW_STATE_TOO_HOT:
		// If oven temperature drops below room temperature
		if (input < TEMPERATURE_ROOM)
		{
			// Ready to reflow
			reflowState = REFLOW_STATE_IDLE;
		}
		break;
		
  case REFLOW_STATE_ERROR:
    // If thermocouple problem is still present
	if (isnan(input))
    {
      // Wait until thermocouple wire is connected
      reflowState = REFLOW_STATE_ERROR; 
    }
    else
    {
      // Clear to perform reflow process
      reflowState = REFLOW_STATE_IDLE; 
    }
    break;    
  }    

  // If switch 1 is pressed
  if (switchStatus == SWITCH_1)
  {
    // If currently reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Button press is for cancelling
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      reflowState = REFLOW_STATE_IDLE;
    }
  } 

  // Simple switch debounce state machine (for switch #1 (both analog & digital
	// switch supported))
  switch (debounceState)
  {
  case DEBOUNCE_STATE_IDLE:
    // No valid switch press
    switchStatus = SWITCH_NONE;
    // If switch #1 is pressed
	if (digitalRead(switch1Pin) == LOW)
	{
		// Intialize debounce counter
		lastDebounceTime = millis();
		// Proceed to check validity of button press
		debounceState = DEBOUNCE_STATE_CHECK;
	}	
    break;

  case DEBOUNCE_STATE_CHECK:
	// If switch #1 is still pressed
	if (digitalRead(switch1Pin) == LOW)
	{
		// If minimum debounce period is completed
		if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
		{
			// Proceed to wait for button release
			debounceState = DEBOUNCE_STATE_RELEASE;
		}
	}
	// False trigger
	else
	{
		// Reinitialize button debounce state machine
		debounceState = DEBOUNCE_STATE_IDLE; 
	}
    break;

  case DEBOUNCE_STATE_RELEASE:
	if (digitalRead(switch1Pin) == HIGH)
	{
      // Valid switch 1 press
      switchStatus = SWITCH_1;
      // Reinitialize button debounce state machine
      debounceState = DEBOUNCE_STATE_IDLE; 
    }
    break;
  }

  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();

    reflowOvenPID.Compute();

    if((now - windowStartTime) > windowSize)
    { 
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if(output > (now - windowStartTime)) {
		// Turn relay on to heat oven
		digitalWrite(ssrPin, HIGH);
		// Turn on LED to indicate oven is powered
		digitalWrite(ledRedPin, HIGH);
	} else {
		// Turn relay off to turn oven off
		digitalWrite(ssrPin, LOW);   
		// Turn LED off to indicate oven is not powered
		digitalWrite(ledRedPin, LOW);
	}
  } else {
	// Reflow oven process is off, ensure oven is off
    digitalWrite(ssrPin, LOW);
	digitalWrite(ledRedPin, LOW);
  }
}

// ##############################################################
// Simple function to clear the Sparkfun serial LCD display
void clearScreen()
{
  //clears the screen, you will use this a lot!
  LCD.write(0xFE);
  LCD.write(0x01); 
}
//-------------------------------------------------------------------------------------------
void selectLineOne()
{ 
  //puts the cursor at line 0 char 0.
  LCD.write(0xFE); //command flag
  LCD.write(128); //position
}
//-------------------------------------------------------------------------------------------
void selectLineTwo()
{ 
  //puts the cursor at line 0 char 0.
  LCD.write(0xFE); //command flag
  LCD.write(192); //position
}
//-------------------------------------------------------------------------------------------
void moveCursorRightOne()
{
  //moves the cursor right one space
  LCD.write(0xFE); //command flag
  LCD.write(20); // 0x14
}
//-------------------------------------------------------------------------------------------
void moveCursorLeftOne()
{
  //moves the cursor left one space
  LCD.write(0xFE); //command flag
  LCD.write(16); // 0x10
}

void backlight(byte val){
   // backlight values: 128 (Off) to 157 (fully on)
   LCD.write(0x7C); // command flag
   LCD.write(val); // backlight setting
}