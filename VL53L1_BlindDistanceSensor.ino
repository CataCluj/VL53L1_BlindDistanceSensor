/*
	Written by Cat Cristea based on public domain/beerware code from Nathan Seidle from SparkFun Electronics
	Date: 2019 - ...
*/

#include <Wire.h>
#include "SparkFun_VL53L1X_Arduino_Library.h"

#define debug 1
#define prescaler 64

#define FREQ_MIN 2
#define FREQ_MAX 20
#if prescaler == 1
	#define OCR_MIN 8000000/FREQ_MAX	// When prescaler is 1
	#define OCR_MAX 8000000/FREQ_MIN
#elif prescaler == 64
	#define OCR_MIN 8000000/(64*FREQ_MAX)	// 12.5(10000Hz) When prescaler is 64. For some reason they become negative
	#define OCR_MAX 8000000/(64*FREQ_MIN)	// 125
#else
	#error Must be one of above
#endif

#define TONE_DELAY 1
#define DIST_MIN 100	//Not exactly mm but ballpark
#define DIST_MAX 3000

VL53L1X distanceSensor;

const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int DistAvg = 5000;				// Start average with something to get there faster
float DistAvgWeight = 0.9;      // IIR a,Weight of new Measurement. Y[n] = aX[n] + (1-a)Y[n-1]

uint16_t Tim1Top = 0;			//This will determine Tone Frequency (Inversely-proportional)

int NewDist = 0;

//Logarithmic: more change near the higher frequencies

void setup(void)
{
	Wire.begin();
	Wire.setClock(400000); //Increase I2C bus speed to 400kHz
#if debug == 1
	Serial.begin(115200);
	Serial.println("VL53L1X");
#endif
//Initialize all readings to 0
	for (int thisReading = 0; thisReading < numReadings; thisReading++)
	{	readings[thisReading] = 0;	}
	if (distanceSensor.begin() == false)
	{	Serial.println("Sensor offline!");	}

// set up 8 MHz timer on CLOCKOUT (OC1A), Pin 9, Port B1
	pinMode (9, OUTPUT); 
	TCCR1A = bit (COM1A0) | bit (WGM11) | bit (WGM10);	// toggle OC1A on Compare Match. WGM for Fast PWM  (WGM13:0 = 15 = 0b1111)
#if prescaler == 1	// OCR1A val of 8000 => 1000Hz; 800 => 10000Hz. Min Freq = 8000000/65535 = 122Hz
	TCCR1B = bit (WGM13) | bit (WGM12) | bit (CS10);	// Fast PWM, no prescaling
#elif prescaler == 64
//	TCCR1B = bit (WGM12) | bit (CS10) | bit (CS11);	// CTC mode, 64 prescaling. OCR1A 512000 => 1000Hz; 51200 => 10000Hz. Min Freq = 2Hz. Max = 
//	TCCR1A	= 0b01000011;	//Making any of COMnX1:0 non-zero overrides GPIO Pin Function. Clear OC1A on Compare Match
//				||||||||
//				||||||--WGM11, WGM10 (11) Fast PWM. Also set WGM13:2 in TCCRnB to 11
//				||||--	Reserved, must be 0
//				||--	COM1B1, COM1B0:
//				--		COM1A1, COM1A0: 01 = Toggle OC1A on Compare Match

	TCCR1B	= 0b00011011;//This is how we start/stop the timer, by changing clock source
//				||||||||
//				|||||---CS12:0 (001). Clock Select clkI/O / 1 (No Prescaling)
//				|||--	WGM13, WGM12 (11) Fast PWM Mode. Also set WGM11:0 in TCCRnB to 00
//				||-		Reserved, must be (0)
//				|-		^ICES1: Input Capture Edge Select (0)		
//				-		^ICNC1: Input Capture Noise Canceler (0)		
#else
	#error Must be one of above
#endif

#if debug == 2	//We toggle GPIO every iteration to see how fast it is
	TCCR1A = 0;	//Clear COM1A0 to return pin to GPIO mode. Also Fast PWM mode is irrelevant
#endif
}

void loop(void)
{
	distanceSensor.startMeasurement(); //Write configuration bytes to initiate measurement
	while (distanceSensor.newDataReady() == false)	//Poll for completion of measurement. Takes 40-50ms.
	{	delay(1);	}
	
	unsigned int uiSignalRate = distanceSensor.getSignalRate();
	if (uiSignalRate > 10)
	{
		NewDist = distanceSensor.getDistance(); 							//Get the result of the measurement from the sensor
		DistAvg = DistAvgWeight * NewDist + (1 - DistAvgWeight) * DistAvg;
	}
	uint16_t ui16_New_OCR = fscale(DIST_MIN, DIST_MAX, OCR_MIN, OCR_MAX, DistAvg, 4);

	OCR1A = ui16_New_OCR;
//	delay(TONE_DELAY);
//	delay(100);

#if debug == 1
	Serial.print("D ");		Serial.print(DistAvg);			//DistAvg. Could be OCR_MIN
	Serial.print(" O ");	Serial.print(ui16_New_OCR);		//ui16_New_OCR. Could be OCR_MAX
	Serial.print(" S ");	Serial.println(uiSignalRate);
#endif

#if debug == 2	//We toggle GPIO every iteration to see how fast it is
//	PINB = 0b00100000;	//Toggle Pin B1. This works but I don't know why "0b00100000" means Bit 1 as in Port B1
//	asm ("sbi %0, %1 \n": : "I" (_SFR_IO_ADDR(PINB)), "I" (PINB1));	//This doesn't work
	digitalWrite(9, !digitalRead(9));	//This works
#endif
}

float fscale( float originalMin, float originalMax, float newBegin, float
              newEnd, float inputValue, float curve)
{
  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  // Check for out of range inputValues
	if (inputValue < originalMin) 
	{	inputValue = originalMin;	}
	if (inputValue > originalMax)
	{	inputValue = originalMax;	}

	
	OriginalRange = originalMax - originalMin;	// Zero Refference the values

	if (newEnd > newBegin)
	{	NewRange = newEnd - newBegin;	}
	else
	{
		NewRange = newBegin - newEnd;
		invFlag = 1;
	}

	zeroRefCurVal = inputValue - originalMin;
	normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
	if (originalMin > originalMax )
	{	return 0;	}

	if (invFlag == 0)
	{	rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;	}
	else     // invert the ranges
	{	rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);	}

	return rangedValue;
}
