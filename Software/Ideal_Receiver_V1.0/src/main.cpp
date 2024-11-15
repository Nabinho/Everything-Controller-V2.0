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

// SSD1306 control libraries
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// BME680 libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// Servo control library
#include <PWMServo.h>
#include <Servo.h>

// Button library
#include <Bounce2.h>

// GPS library
#include <TinyGPS++.h>

// Teensy ADC libraries
#include <ADC.h>
#include <ADC_util.h>

// ********************** Project Macros **********************
// Macro to disable or enable serial debugging
// #define DEBUG_ANALOG
// #define DEBUG_DIGITAL
// #define DEBUG_ENCODER
// #define DEBUG_BUTTONS
// #define DEBUG_AMBIENT
// #define DEBUG_GPS
// #define DEBUG_BATTERY

// PWM output pins configuration
#define PIN_PWM1 8
#define PIN_PWM2 7
#define PIN_PWM3 6
#define PIN_PWM4 5
#define PIN_PWM5 4
#define PIN_PWM6 3
#define PIN_PWM7 20
#define PIN_PWM8 21

// PPM output pins configuration
#define PIN_PPM1 22
#define PIN_PPM2 23

// Digital output pins configuration
#define PIN_DOUT1 29
#define PIN_DOUT2 30
#define PIN_DOUT3 31
#define PIN_DOUT4 28
#define PIN_DOUT5 27
#define PIN_DOUT6 26
#define PIN_DOUT7 25
#define PIN_DOUT8 24

// LED output pins configuration
#define PIN_LED1 32
#define PIN_LED2 33

// Buttons pins configuration
#define PIN_BTN1 14
#define PIN_BTN2 15
#define PIN_BTN3 16
#define PIN_BTN4 17

// NRF24L01 pins definitions
#define PIN_CE 9
#define PIN_CSN 10

// SSD1306 display definitions
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define DISPLAY_ADDRESS 0x3C
#define DISPLYA_RESET -1

// ********************** Project Objects *********************
// Teensy analog pins object
ADC *adc = new ADC();

// NRF24L01 control object
RF24 radio(PIN_CE, PIN_CSN);

// SSD1306 display control object
Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, DISPLYA_RESET);

// BME680 control object
Adafruit_BME680 ambient(&Wire);

// Servo control objects
PWMServo SERVOS[2];

// The TinyGPSPlus object
TinyGPSPlus gps;

// ********************** Project Variables *******************
// PWM pins definitions
const uint8_t NUMBER_PWM = 8;
const uint8_t PWM_PINS[NUMBER_PWM] = {PIN_PWM1, PIN_PWM2, PIN_PWM3, PIN_PWM4, PIN_PWM5, PIN_PWM6, PIN_PWM7, PIN_PWM8};
uint8_t PWM_OUT[NUMBER_PWM] = {0, 0, 0, 0, 0, 0, 0, 0};

// PPM pins definitions
const uint8_t NUMBER_PPM = 2;
const uint8_t PPM_PINS[NUMBER_PPM] = {PIN_PPM1, PIN_PPM2};
uint16_t PPM_OUT[NUMBER_PPM] = {90, 90};

// Digital pins definitions
const uint8_t NUMBER_DOUTS = 8;
const uint8_t DOUT_PINS[NUMBER_DOUTS] = {PIN_DOUT1, PIN_DOUT2, PIN_DOUT3, PIN_DOUT4, PIN_DOUT5, PIN_DOUT6, PIN_DOUT7, PIN_DOUT8};
bool DOUT_OUT[NUMBER_DOUTS] = {0, 0, 0, 0, 0, 0, 0, 0};

// LEDs Pins
const uint8_t NUMBER_LEDS = 2;
const uint8_t LED_PINS[NUMBER_LEDS] = {PIN_LED1, PIN_LED2};

// Controller buttons objects
const uint8_t NUMBER_BUTTONS = 4;
const uint8_t DEBOUNCE_INTERVAL = 50;
Bounce BUTTONS[NUMBER_BUTTONS];
const uint8_t PINS_BUTTONS[NUMBER_BUTTONS] = {PIN_BTN1, PIN_BTN2, PIN_BTN3, PIN_BTN4};
bool buttons_readings[NUMBER_BUTTONS] = {LOW, LOW, LOW, LOW};
bool last_buttons_readings[NUMBER_BUTTONS] = {HIGH, HIGH, HIGH, HIGH};

// Radio address and channel
const uint8_t radio_CHANNEL = 71;
const uint8_t radio_ADDRESSES[2][6] = {{0xB6, 0xE9, 0x00, 0x7A, 0x43}, {0x10, 0xCA, 0x87, 0x3F, 0xD8}};
const bool radio_NUMBER = 1;

// Message receiving variables
uint8_t channel;
uint8_t bytes;
unsigned long last_message = 0;
const uint16_t RX_TIMEOUT = 2500; // [ms]

// Variables for display timing
unsigned long display_time = 0;
const uint16_t UPDATE_INTERVAL = 25; // [ms]

// Ambient sensor variables
const float sea_level_pressure = 1013.25; // [hpa]
float temperature_reading = 0.0;
float humidity_reading = 0.0;
float pressure_reading = 0.0;
float air_quality_reading = 0.0;
float altitude_reading = 0.0;
String temperature_str;
String humidity_str;
String pressure_str;
String altitude_str;
unsigned long ambient_time = 0;
const uint16_t READING_INTERVAL = 1000; // [ms]

// Distance reference variables
double reference_lng = 0;
double reference_lat = 0;
double new_lng = 0;
double new_lat = 0;
float distance = 0;
bool update_reference = true;
unsigned long gps_time = 0;
const uint16_t GPS_INTERVAL = 1000; // [ms]

// Battery Voltage Divider Variables
const uint8_t BATTERY_PIN = A14;
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

// Controller variables structure
typedef struct
{
  bool buttons_message[8];
  uint16_t analogs_message[8];
  int16_t encoders_messsage[2];
} controller_variables;
controller_variables controller;
enum ANALOG_MESSAGE
{
  JOY_XL,
  JOY_YL,
  JOY_XR,
  JOY_YR,
  ROT_L,
  ROT_R,
  SLI_L,
  SLI_R
};
enum BUTTONS_MESSAGE
{
  JOY_L,
  JOY_R,
  ENC_L,
  ENC_R,
  MX_EL,
  MX_CL,
  MC_CR,
  MX_ER
};
enum ENCODERS_MESSAGE
{
  LEFT,
  RIGHT
};

// ********************** Project Functions *******************
// Display controller button state
void drawButtonState(int x, int y, int width, int height, bool state)
{
  display.drawRoundRect(x, y, width, height, 5, WHITE);
  if (state)
  {
    display.fillRoundRect(x + 2, y + 2, width - 4, height - 4, 2, WHITE);
  }
}

// Display controller joystick position
void drawJoyPosition(int x, int y, int radius, int px, int py)
{
  display.drawLine(x, (y + radius) - 4, x, (y - radius) + 4, WHITE);
  display.drawLine((x + radius) - 4, y, (x - radius) + 4, y, WHITE);
  display.drawRoundRect(x - radius, y - radius, (radius * 2) + 1, (radius * 2) + 1, 5, WHITE);
  display.drawCircle(x, y, radius, WHITE);
  display.fillCircle(px, py, radius / 10, WHITE);
}

// Display controller potentiometer position
void drawPottentiometerBar(int x, int y, int width, int height, int progress)
{
  float bar = ((float)(height - 4) / 100) * progress;
  display.drawRoundRect(x, y, width, height, 5, WHITE);
  display.fillRoundRect(x + 2, y + 2, width - 4, height - 4, 2, WHITE);
  display.fillRoundRect(x + 2, y + 2, width - 4, bar, 2, BLACK);
}

// Function to display all controller inputs
void display_controller()
{
  display.clearDisplay();
  drawJoyPosition(22, 22, 22, map(controller.analogs_message[JOY_XL], 0, 4095, 2, 42), map(controller.analogs_message[JOY_YL], 0, 4095, 2, 42));
  drawPottentiometerBar(46, 0, 6, 44, map(controller.analogs_message[ROT_L], 0, 4095, 0, 100));
  drawPottentiometerBar(54, 0, 6, 44, map(controller.analogs_message[SLI_L], 0, 4095, 100, 0));
  drawPottentiometerBar(61, 0, 6, 22, map(controller.encoders_messsage[LEFT], -90, 90, 0, 100));
  drawPottentiometerBar(61, 22, 6, 22, map(controller.encoders_messsage[RIGHT], -90, 90, 0, 100));
  drawPottentiometerBar(68, 0, 6, 44, map(controller.analogs_message[SLI_R], 0, 4095, 100, 0));
  drawPottentiometerBar(76, 0, 6, 44, map(controller.analogs_message[ROT_R], 0, 4095, 0, 100));
  drawJoyPosition(105, 22, 22, map(controller.analogs_message[JOY_XR], 0, 4095, 86, 126), map(controller.analogs_message[JOY_YR], 0, 4095, 2, 42));
  drawButtonState(0, 46, 32, 8, !controller.buttons_message[JOY_L]);
  drawButtonState(32, 46, 32, 8, !controller.buttons_message[ENC_L]);
  drawButtonState(64, 46, 32, 8, !controller.buttons_message[ENC_R]);
  drawButtonState(96, 46, 32, 8, !controller.buttons_message[JOY_R]);
  drawButtonState(0, 56, 32, 8, !controller.buttons_message[MX_EL]);
  drawButtonState(32, 56, 32, 8, !controller.buttons_message[MX_CL]);
  drawButtonState(64, 56, 32, 8, !controller.buttons_message[MC_CR]);
  drawButtonState(96, 56, 32, 8, !controller.buttons_message[MX_ER]);
  display.display();
}

// Function to display ambient readings
void display_ambient()
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("TEM:" + temperature_str);
  display.setCursor(0, 16);
  display.print("HUM:" + humidity_str);
  display.setCursor(0, 32);
  display.print("PRE:" + pressure_str);
  display.setCursor(0, 48);
  display.print("ALT:" + altitude_str);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(110, 4);
  display.print("*C");
  display.setCursor(110, 20);
  display.print("%");
  display.setCursor(110, 36);
  display.print("hPa");
  display.setCursor(110, 52);
  display.print("m");
  display.display();
}

// Function to display receiver outputs
void display_output()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  for (uint8_t index = 0; index < NUMBER_PWM; index++)
  {
    display.setCursor(0, index * 8);
    display.print("PWM" + String(index) + ":" + String(PWM_OUT[index]));
  }
  for (uint8_t index = 0; index < NUMBER_PWM; index++)
  {
    display.setCursor(50, index * 8);
    display.print("D" + String(index) + ":" + String(DOUT_OUT[index]));
  }
  for (uint8_t index = 0; index < NUMBER_PPM; index++)
  {
    display.setCursor(80, index * 8);
    display.print("PPM" + String(index) + ":" + String(PPM_OUT[index]));
  }
  display.display();
}

// Function to display the distance between reference position and new position
void display_gps()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("DISTANCE(m):");
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 8);
  display.print(distance);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 40);
  display.print("REFERENCE:");
  display.setCursor(0, 48);
  display.print("LAT:" + String(reference_lat,2));
  display.setCursor(0, 56);
  display.print("LNG:" + String(reference_lng,2));
  display.display();
  display.setCursor(64, 40);
  display.print("POSITION:");
  display.setCursor(64, 48);
  display.print("LAT:" + String(new_lat,2));
  display.setCursor(64, 56);
  display.print("LNG:" + String(new_lng,2));
  display.display();
}

// ********************** Project Configuration ***************
void setup()
{

  // Initializes serial communication if macro is enabled
#if defined(DEBUG_ANALOG) || defined(DEBUG_DIGITAL) || defined(DEBUG_ENCODER) || defined(DEBUG_BUTTONS) || defined(DEBUG_AMBIENT) || defined(DEBUG_GPS) || defined(DEBUG_BATTERY)
  Serial.begin(115200);
#endif
  Serial1.begin(9600);

  // Initializes and configures the PWM pins
  for (uint8_t index = 0; index < NUMBER_PWM; index++)
  {
    pinMode(PWM_PINS[index], OUTPUT);
    analogWrite(PWM_PINS[index], PWM_OUT[index]);
  };

  // Initializes and configures the PPM pins
  for (uint8_t index = 0; index < NUMBER_PPM; index++)
  {
    SERVOS[index].attach(PPM_PINS[index]);
    SERVOS[index].write(PPM_OUT[index]);
  }

  // Initializes and configures the PWM pins
  for (uint8_t index = 0; index < NUMBER_DOUTS; index++)
  {
    pinMode(DOUT_PINS[index], OUTPUT);
    digitalWrite(DOUT_PINS[index], DOUT_OUT[index]);
  }

  // Initializes the LEDs
  for (uint8_t index = 0; index < NUMBER_LEDS; index++)
  {
    pinMode(LED_PINS[index], OUTPUT);
    digitalWrite(LED_PINS[index], LOW);
  }

  // Digital pins configuration
  for (uint8_t index = 0; index < NUMBER_BUTTONS; index++)
  {
    BUTTONS[index].attach(PINS_BUTTONS[index], INPUT);
    BUTTONS[index].interval(DEBOUNCE_INTERVAL);
  }

  // Configures Teensy ADC0
	adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
	adc->adc0->setAveraging(32);
	adc->adc0->setResolution(12);
	adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
	adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);

  // Battery Voltage Pin Configuration
	pinMode(BATTERY_PIN, INPUT_DISABLE);

  // Initializes and configures the display
  display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS);
  display.clearDisplay();
  display.display();

  // Initializes the ambient sensor
  ambient.begin();
  ambient.setTemperatureOversampling(BME680_OS_8X);
  ambient.setHumidityOversampling(BME680_OS_2X);
  ambient.setPressureOversampling(BME680_OS_4X);
  ambient.setIIRFilterSize(BME680_FILTER_SIZE_3);
  ambient.setGasHeater(320, 150);

  // Initializes and configures the radio
  radio.begin();
  radio.setChannel(radio_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setPayloadSize(sizeof(controller));
  radio.openWritingPipe(radio_ADDRESSES[radio_NUMBER]);
  radio.openReadingPipe(1, radio_ADDRESSES[!radio_NUMBER]);
  radio.startListening();
}

// ********************** Project Main Loop ***************
void loop()
{

  // ---------------------- Analog Readings --------------------
	// Reads Controller Battery Voltage
	if ((millis() - last_battery) > BATTERY_READING)
	{
		VD_VOUT = (adc->adc0->analogRead(BATTERY_PIN) * 3.3) / 4095.0;
		VD_VIN = (VD_VOUT / (VD_R2 / (VD_R1 + VD_R2))) - 0.12;
		if ((VD_VIN > last_VD_VIN + HYSTERESIS) || (VD_VIN < last_VD_VIN - HYSTERESIS))
		{
			analogWrite(LED_PINS[1], map(VD_VIN, MIN_VIN, 7.3, 0, 255));
			last_VD_VIN = VD_VIN;
		}
		last_battery = millis();
	}

#ifdef DEBUG_BATTERY
		Serial.print(VD_VIN);
		Serial.print(" | ");
		Serial.print(last_VD_VIN);
		Serial.println("");
#endif

  // Reads Receiver Buttons
  for (uint8_t index = 0; index < NUMBER_BUTTONS; index++)
  {
    BUTTONS[index].update();
    // buttons_readings[index] = BUTTONS[index].read();
    if (BUTTONS[index].read() != last_buttons_readings[index])
    {
      last_buttons_readings[index] = BUTTONS[index].read();
      if (BUTTONS[index].read() == LOW)
      {
        buttons_readings[index] = !buttons_readings[index];
        display.clearDisplay();
        display.display();
      }
    }
#ifdef DEBUG_BUTTONS
    Serial.print(buttons_readings[index]);
    Serial.print(" | ");
#endif
  }
#ifdef DEBUG_BUTTONS
  Serial.println("");
#endif

  if (radio.available(&channel))
  {
    bytes = radio.getPayloadSize();
    radio.read(&controller, bytes);
    digitalWrite(LED_PINS[0], HIGH);
#ifdef DEBUG_ANALOG
    for (uint8_t index = 0; index < 8; index++)
    {
      Serial.print(controller.analogs_message[index]);
      Serial.print(" | ");
    }
    Serial.println("");
#endif
#ifdef DEBUG_DIGITAL
    for (uint8_t index = 0; index < 8; index++)
    {
      Serial.print(controller.buttons_message[index]);
      Serial.print(" | ");
    }
    Serial.println("");
#endif
#ifdef DEBUG_ENCODER
    for (uint8_t index = 0; index < 2; index++)
    {
      Serial.print(controller.encoders_messsage[index]);
      Serial.print(" | ");
    }
    Serial.println("");
#endif
    last_message = millis();
  }
  else if ((millis() - last_message) > RX_TIMEOUT)
  {
    digitalWrite(LED_PINS[0], LOW);
    for (uint8_t index = 0; index < NUMBER_PWM; index++)
    {
      analogWrite(PWM_PINS[index], 0);
    }
    for (uint8_t index = 0; index < NUMBER_PPM; index++)
    {
      SERVOS[index].write(90);
    }
    for (uint8_t index = 0; index < NUMBER_DOUTS; index++)
    {
      digitalWrite(DOUT_PINS[index], 0);
    }
  }

  if (buttons_readings[0])
  {
    if (update_reference)
    {
      while (Serial1.available() > 0)
      {
        gps.encode(Serial1.read());
        reference_lng = gps.location.lng();
        reference_lat = gps.location.lat();
#ifdef DEBUG_GPS
        Serial.print("Ref. Latitude= ");
        Serial.print(reference_lat, 6);
        Serial.print("| Ref. Longitude= ");
        Serial.println(reference_lng, 6);
#endif
        update_reference = false;
      }
    }
    else
    {
      while (Serial1.available() > 0)
      {
        gps.encode(Serial1.read());
        if (gps.location.isUpdated())
        {
          new_lng = gps.location.lng();
          new_lat = gps.location.lat();
          distance = gps.distanceBetween(reference_lat, reference_lng, new_lat, new_lng);
#ifdef DEBUG_GPS
          Serial.print("Ref. Latitude= ");
          Serial.print(reference_lat, 6);
          Serial.print(" | Ref. Longitude= ");
          Serial.print(reference_lng, 6);
          Serial.print(" | New Latitude= ");
          Serial.print(reference_lat, 6);
          Serial.print(" | New Longitude= ");
          Serial.print(reference_lng, 6);
          Serial.print(" | Distance= ");
          Serial.print(distance, 6);
#endif
        }
      }
    }
    if ((millis() - gps_time) > GPS_INTERVAL)
    {
      display_gps();
      gps_time = millis();
    }
  }
  else if (buttons_readings[1])
  {
    if ((millis() - display_time) > UPDATE_INTERVAL)
    {
      display_controller();
      display_time = millis();
    }
  }
  else if (buttons_readings[2])
  {
    if ((millis() - ambient_time) > READING_INTERVAL)
    {

      // Reads ambient sensor
      ambient.performReading();
      temperature_reading = ambient.temperature;
      humidity_reading = ambient.humidity;
      pressure_reading = ambient.pressure / 100.00;
      air_quality_reading = ambient.gas_resistance / 1000.00;
      altitude_reading = ambient.readAltitude(sea_level_pressure);
      temperature_str = String(temperature_reading, 1);
      humidity_str = String(humidity_reading, 1);
      pressure_str = String(pressure_reading, 1);
      altitude_str = String(altitude_reading, 1);

#ifdef DEBUG_AMBIENT
      Serial.print("Temperature = ");
      Serial.print(temperature_reading);
      Serial.println(" *C");
      Serial.print("Pressure = ");
      Serial.print(pressure_reading);
      Serial.println(" hPa");
      Serial.print("Humidity = ");
      Serial.print(humidity_reading);
      Serial.println(" %");
      Serial.print("Air Quality = ");
      Serial.print(air_quality_reading);
      Serial.println(" KOhms");
      Serial.print("Altitude = ");
      Serial.print(altitude_reading);
      Serial.println(" m");
      Serial.println();
#endif

      // Updates timing
      ambient_time = millis();
      display_ambient();
    }
  }
  else if (buttons_readings[3])
  {
    for (uint8_t index = 0; index < NUMBER_PWM; index++)
    {
      PWM_OUT[index] = map(controller.analogs_message[index], 0, 4095, 0, 254);
      analogWrite(PWM_PINS[index], PWM_OUT[index]);
    }
    for (uint8_t index = 0; index < NUMBER_PPM; index++)
    {
      PPM_OUT[index] = map(controller.encoders_messsage[index], -180, 180, 0, 180);
      SERVOS[index].write(PPM_OUT[index]);
    }
    for (uint8_t index = 0; index < NUMBER_DOUTS; index++)
    {
      DOUT_OUT[index] = !controller.buttons_message[index];
      digitalWrite(DOUT_PINS[index], DOUT_OUT[index]);
    }
    display_output();
  }
  else if (!buttons_readings[0])
  {
    update_reference = true;
  }
  else if (!buttons_readings[3])
  {
    for (uint8_t index = 0; index < NUMBER_PWM; index++)
    {
      analogWrite(PWM_PINS[index], 0);
    }
    for (uint8_t index = 0; index < NUMBER_PPM; index++)
    {
      SERVOS[index].write(90);
    }
    for (uint8_t index = 0; index < NUMBER_DOUTS; index++)
    {
      digitalWrite(DOUT_PINS[index], 0);
    }
  }
}
// ************************************************************
