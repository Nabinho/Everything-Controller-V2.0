/**************************************************************
 * Everything Controller (v2.0)
 *
 * Base code for the "Everything Controller V2.0" to send data to a receiver.
 * Communication is based on RF24 using the NRF24L01+ module. The controller
 * sends data from 8 digital buttons, 2 joysticks (2 X-axis and 2 Y-axis),
 * two sliders, two rotary potentiometers and two rotary encoders.
 * With this data, it is possible to control anything.
 *
 * Written by Giovanni de Castro (10/03/2024).
 *************************************************************/

// ********************** Project Libraries *******************
// Arduino framework
#include <Arduino.h>

// NRF24L01 control libraries
#include <SPI.h>
#include <RF24.h>

// SSD1327 control libraries
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1327.h>
#include "icons.h"

// Teensy ADC libraries
#include <ADC.h>
#include <ADC_util.h>

// Moving average library
#include <movingAvg.h>

// Encoder library
#include <Encoder.h>

// Button library
#include <Bounce2.h>

// ********************** Project Macros **********************
// Macro to disable or enable serial debugging
// #define DEBUG_BATTERY
// #define DEBUG_ANALOG
// #define DEBUG_DIGITAL
// #define DEBUG_ENCODER
// #define DEBUG_GENERIC // used to debug logic without the controller readings

// Controller pins definitions (Analog Inputs)
#define PIN_JXAL A6	 // Joystick X Axis Left
#define PIN_JYAL A7	 // Joystick Y Axis Left
#define PIN_JXAR A9	 // Joystick X Axis Right
#define PIN_JYAR A8	 // Joystick Y Axis Right
#define PIN_RPOTL A3 // Rotary Potentiometer Left
#define PIN_SPOTL A2 // Slider Potentiometer Left
#define PIN_RPOTR A1 // Rotary Potentiometer Right
#define PIN_SPOTR A0 // Slider Potentiometer Right

// Controller pins definitions (Digital Inputs)
#define PIN_JBTL 0	  // Joystick Button Left
#define PIN_JBTR 1	  // Joystick Button Right
#define PIN_EBTL 2	  // Encoder Button Left
#define PIN_EBTR 3	  // Encoder Button Right
#define PIN_ECLKL 4	  // Encoder CLK Left
#define PIN_EDTL 5	  // Encoder DT Left
#define PIN_EDTR 6	  // Encoder DT Right
#define PIN_ECLKR 7	  // Encoder CLK Right
#define PIN_MXBTEL 24 // Cherry MX Button Left
#define PIN_MXBTCL 33 // Cherry MX Button Center Left
#define PIN_MXBTER 31 // Cherry MX Button Right
#define PIN_MXBTCR 26 // Cherry MX Button Center Right
#define PIN_DIPER 29  // DIP Switch 1
#define PIN_DIPCR 30  // DIP Switch 2
#define PIN_DIPCL 28  // DIP Switch 3
#define PIN_DIPEL 27  // DIP Switch 4

// Controller pins definitions (LEDs Outputs)
#define PIN_LED1 25 // RF Message Feedback LED
#define PIN_LED2 32 // Battery Level Feedback LED | Vibracall Motor

// NRF24L01 pins definitions
#define PIN_CE 9
#define PIN_CSN 10

// SSD1327 display Display definitions
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 128
#define DISPLAY_ADDRESS 0x3D
#define DISPLYA_RESET -1

// ********************** Project Objects *********************
// Controller encoder objects
const uint8_t NUMBER_KNOBS = 2;
Encoder KNBOS[NUMBER_KNOBS] = {
	Encoder(PIN_ECLKL, PIN_EDTL),
	Encoder(PIN_ECLKR, PIN_EDTR)};

// Controller buttons objects
const uint8_t NUMBER_BUTTONS = 8;
const uint8_t DEBOUNCE_INTERVAL = 50;
Bounce BUTTONS[NUMBER_BUTTONS];
const uint8_t PINS_BUTTONS[NUMBER_BUTTONS] = {PIN_JBTL, PIN_JBTR, PIN_EBTL, PIN_EBTR, PIN_MXBTEL, PIN_MXBTCL, PIN_MXBTCR, PIN_MXBTER};
bool buttons_readings[NUMBER_BUTTONS];
bool MXbuttons_states[NUMBER_BUTTONS / 2] = {false, false, false, false};

// MX SW Latch Switches Pins
const uint8_t DIP_PINS[NUMBER_BUTTONS / 2] = {PIN_DIPEL, PIN_DIPCL, PIN_DIPCR, PIN_DIPER};
bool MX_latch[NUMBER_BUTTONS / 2];
bool latch_change[NUMBER_BUTTONS / 2] = {true, true, true, true};

// Analog Pins
const uint8_t NUMBER_ANALOG_INS = 8;
const uint8_t ANALOG_PINS[NUMBER_ANALOG_INS] = {PIN_JXAL, PIN_JYAL, PIN_JXAR, PIN_JYAR, PIN_RPOTL, PIN_RPOTR, PIN_SPOTL, PIN_SPOTR};
uint16_t analog_readings[NUMBER_ANALOG_INS];
uint16_t averages_readings[NUMBER_ANALOG_INS];
const uint8_t NUMBER_SAMPLES = 10;
movingAvg analog_averages[NUMBER_ANALOG_INS] = {
	movingAvg(NUMBER_SAMPLES),
	movingAvg(NUMBER_SAMPLES),
	movingAvg(NUMBER_SAMPLES),
	movingAvg(NUMBER_SAMPLES),
	movingAvg(NUMBER_SAMPLES),
	movingAvg(NUMBER_SAMPLES),
	movingAvg(NUMBER_SAMPLES),
	movingAvg(NUMBER_SAMPLES)};

// LEDs Pins
const uint8_t NUMBER_LEDS = 2;
const uint8_t LED_PINS[NUMBER_LEDS] = {PIN_LED1, PIN_LED2};

// NRF24L01 control object
RF24 radio(PIN_CE, PIN_CSN);

// SSD1327 display control object
Adafruit_SSD1327 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, DISPLYA_RESET, 3400000);

// Teensy analog pins object
ADC *adc = new ADC();

// ********************** Project Variables *******************
// Radio address and channel
const uint8_t radio_CHANNEL = 71;
const uint8_t radio_ADDRESSES[2][6] = {{0xB6, 0xE9, 0x00, 0x7A, 0x43}, {0x10, 0xCA, 0x87, 0x3F, 0xD8}};
const bool radio_NUMBER = 0;

// Variables for failsafe and display update
bool message_acknowledged;
bool last_acknowledgement;
bool vibration_animation = true;
uint8_t vibration_PWM = 250;
bool signal_state = false;

// Encoders reading variables
int16_t new_positions[NUMBER_KNOBS] = {0, 0};
int16_t last_positions[NUMBER_KNOBS] = {0, 0};

// Variables for radio timings
unsigned long last_message = 0;
const uint8_t TX_INTERVAL = 10; // [ms] | 1 ms to acknowlegement (6 ms total)
unsigned long last_message_sent = 0;
const uint16_t TX_TIMEOUT = 2500; // [ms] | FAILSAFE

// Battery Voltage Divider Variables
const uint8_t BATTERY_PIN = A14;
const uint8_t NUMBER_VREADS = 50;
float VD_VOUT = 0.0;
float VD_VIN = 0.0;
float last_VD_VIN = 0.0;
const float VD_R1 = 51000.0;
const float VD_R2 = 33000.0;
const float MIN_VIN = 6.5;
const float HYSTERESIS = 0.1;
unsigned long last_battery = 5000;
const uint16_t BATTERY_READING = 5000; // [ms]
uint8_t battery_icon = 4;
uint8_t last_battery_icon = 0;
uint16_t average_battery = 0;
movingAvg battery_readings(NUMBER_VREADS);

// Boolean to update OLED Display
bool update_display = true;

// Controller variables structure
typedef struct
{
	bool buttons_message[8];
	uint16_t analogs_message[8];
	int16_t encoders_messsage[2];
} controller_variables;
controller_variables controller;

// ********************** Project Configuration ***************
void setup()
{

	// Initializes serial communication if macro is enabled
#if defined(DEBUG_ANALOG) || defined(DEBUG_DIGITAL) || defined(DEBUG_ENCODER) || defined(DEBUG_BATTERY) || defined(DEBUG_GENERIC)
	Serial.begin(115200);
#endif

	for (uint8_t index = 0; index < NUMBER_LEDS; index++)
	{
		pinMode(LED_PINS[index], OUTPUT);
		digitalWrite(LED_PINS[index], LOW);
	}

	// Initializes and configures the display
	display.begin(DISPLAY_ADDRESS);
	display.clearDisplay();
	display.display();

	// Initializes and configures the radio
	radio.begin();
	radio.setChannel(radio_CHANNEL);
	radio.setDataRate(RF24_250KBPS);
	radio.setPALevel(RF24_PA_MAX);
	radio.setPayloadSize(sizeof(controller));
	radio.openWritingPipe(radio_ADDRESSES[radio_NUMBER]);
	radio.openReadingPipe(1, radio_ADDRESSES[!radio_NUMBER]);
	radio.stopListening();

	// Configures Teensy ADC0
	adc->adc0->setReference(ADC_REFERENCE::REF_EXT);
	adc->adc0->setAveraging(32);
	adc->adc0->setResolution(12);
	adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
	adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);

	// Analog pins configuration
	for (uint8_t index = 0; index < NUMBER_ANALOG_INS; index++)
	{
		pinMode(ANALOG_PINS[index], INPUT_DISABLE);
		analog_averages[index].begin();
	}

	// Battery Voltage Pin Configuration
	pinMode(BATTERY_PIN, INPUT_DISABLE);
	battery_readings.begin();

	// Digital pins configuration
	for (uint8_t index = 0; index < NUMBER_BUTTONS; index++)
	{
		BUTTONS[index].attach(PINS_BUTTONS[index], INPUT);
		BUTTONS[index].interval(DEBOUNCE_INTERVAL);
	}

	// DIP Switch pins configuration
	for (uint8_t index = 0; index < NUMBER_BUTTONS / 2; index++)
	{
		pinMode(DIP_PINS[index], INPUT_PULLUP);
	}

	// Encoders initial value configuration
	for (uint8_t index = 0; index < NUMBER_KNOBS; index++)
	{
		KNBOS[index].write(last_positions[index]);
	}
}

// ********************** Project Main Loop ***************
void loop()
{

	// ---------------------- Analog Readings --------------------
	// Reads Controller Battery Voltage
	if ((millis() - last_battery) > BATTERY_READING)
	{
		average_battery = battery_readings.reading(adc->adc0->analogRead(BATTERY_PIN));
		VD_VOUT = (average_battery * 3.3) / 4095.0;
		VD_VIN = (VD_VOUT / (VD_R2 / (VD_R1 + VD_R2))) - 0.12;
		if ((VD_VIN > last_VD_VIN + HYSTERESIS) || (VD_VIN < last_VD_VIN - HYSTERESIS))
		{
			analogWrite(LED_PINS[1], map(VD_VIN, MIN_VIN, 7.3, 0, 255));
			battery_icon = map(VD_VIN, MIN_VIN, 7.5, 0, 4);
			if (battery_icon != last_battery_icon)
			{
				update_display = true;
				last_battery_icon = battery_icon;
			}
			last_VD_VIN = VD_VIN;
		}
		last_battery = millis();
	}

#ifdef DEBUG_BATTERY
	Serial.print(VD_VIN);
	Serial.print(" | ");
	Serial.print(last_VD_VIN);
	Serial.print(" | ");
	Serial.print(battery_icon);
	Serial.print(" | ");
	Serial.print(last_battery_icon);
#endif

	// Reads Controller Analog Inputs
	for (uint8_t index = 0; index < NUMBER_ANALOG_INS; index++)
	{
		analog_readings[index] = adc->adc0->analogRead(ANALOG_PINS[index]);
		averages_readings[index] = analog_averages[index].reading(analog_readings[index]);
		controller.analogs_message[index] = averages_readings[index];
#ifdef DEBUG_ANALOG
		Serial.print(" | ");
		Serial.print(analog_readings[index]);
		Serial.print(" | ");
#endif
	}

	// ---------------------- Digital Readings --------------------
	// Reads DIP Switch pins
	for (uint8_t index = 0; index < NUMBER_BUTTONS / 2; index++)
	{
		MX_latch[index] = digitalRead(DIP_PINS[index]);
#ifdef DEBUG_DIGITAL
		Serial.print(MX_latch[index]);
		Serial.print(" | ");
#endif
	}
#ifdef DEBUG_DIGITAL
	Serial.println("");
#endif

	// Reads Controller Buttons
	for (uint8_t index = 0; index < NUMBER_BUTTONS / 2; index++)
	{
		BUTTONS[index].update();
		buttons_readings[index] = BUTTONS[index].read();
		controller.buttons_message[index] = buttons_readings[index];
#ifdef DEBUG_DIGITAL
		Serial.print(buttons_readings[index]);
		Serial.print(" | ");
#endif
	}

	// Reads Controller Cherry MC Buttons
	for (uint8_t index = NUMBER_BUTTONS / 2; index < NUMBER_BUTTONS; index++)
	{
		BUTTONS[index].update();
		buttons_readings[index] = BUTTONS[index].read();
		if (MX_latch[index - 4] == LOW)
		{
			if (buttons_readings[index] != latch_change[index - 4])
			{
				latch_change[index - 4] = buttons_readings[index];
				if (buttons_readings[index] == LOW)
				{
					MXbuttons_states[index - 4] = !MXbuttons_states[index - 4];
					controller.buttons_message[index] = MXbuttons_states[index - 4];
				}
			}
		}
		else
		{
			buttons_readings[index] = BUTTONS[index].read();
			controller.buttons_message[index] = buttons_readings[index];
		}
#ifdef DEBUG_DIGITAL
		Serial.print(buttons_readings[index]);
		Serial.print(" | ");
#endif
	}

	// Reads controller encoder position
	for (uint8_t index = 0; index < NUMBER_KNOBS; index++)
	{
		new_positions[index] = -KNBOS[index].read();
		if (new_positions[index] != last_positions[index])
		{
			if (new_positions[index] > 180)
			{
				new_positions[index] = 180;
				KNBOS[index].write(-new_positions[index]);
			}
			if (new_positions[index] < -180)
			{
				new_positions[index] = -180;
				KNBOS[index].write(-new_positions[index]);
			}
			last_positions[index] = new_positions[index];
			controller.encoders_messsage[index] = new_positions[index];
		}
#ifdef DEBUG_ENCODER
		Serial.print(new_positions[index]);
		Serial.print(" | ");
#endif
	}

	// ---------------------- Radio Transmission ------------------
	if ((millis() - last_message) > TX_INTERVAL)
	{
		// Sends the data and checks for acknowledgement
		message_acknowledged = radio.write(&controller, sizeof(controller));
		if (message_acknowledged)
		{
			signal_state = true;
			digitalWrite(LED_PINS[0], HIGH);
			last_message_sent = millis();
		}
		else if ((millis() - last_message_sent) > TX_TIMEOUT)
		{
			signal_state = false;
			update_display = true;
			digitalWrite(LED_PINS[0], LOW);
		}
		if (last_acknowledgement != message_acknowledged)
		{
			update_display = true;
			last_acknowledgement = message_acknowledged;
		}
		last_message = millis();
	}

	// Checks if need to update OLED Display
	if (update_display)
	{
		display.clearDisplay();
		display.drawBitmap(0, 6, SIGNAL_ICONS[signal_state], SIGNAL_SIZE, SIGNAL_SIZE, SSD1327_WHITE);
		display.drawBitmap(80, 0, BATTERY_ICONS[battery_icon], BATTERY_SIZE, BATTERY_SIZE, SSD1327_WHITE);
		display.drawBitmap(19, 32, epd_bitmap_line_icon_for_turnip_vector, 90, 90, SSD1327_WHITE);
		display.display();
		update_display = false;
	}

#if defined(DEBUG_ANALOG) || defined(DEBUG_DIGITAL) || defined(DEBUG_ENCODER) || defined(DEBUG_BATTERY)
	Serial.println("");
#endif
}
// ************************************************************
