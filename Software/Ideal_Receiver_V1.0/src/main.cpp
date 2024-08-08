/**************************************************************
 * Everything Controller - Ideal Receiver (v1.0)
 *
 * Base code for the "Ideal Receiver V1.0" to receive data from
 * my "Everything Controler V2.0", and then control outputs,
 * display the data incoming, display the reading from an IMU
 * and display the distance between two coordinates using a GPS.
 * This final function helps with the monitoring of the 
 * communication distance between both ends.
 *
 * Written by Giovanni de Castro (10/08/2024).
 *************************************************************/

// ********************** Project Libraries *******************
// Arduino framework
#include <Arduino.h>

// Redefinition from RF24 library for faster communication
#define RF24_SPI_SPEED 16000000

// NRF24L01 control libraries
#include <SPI.h>
#include <RF24.h>

// Button reading library
#include <Bounce2.h>

// I2C communication library
#include <Wire.h>

// Display control library
#include <Adafruit_SSD1306.h>

// IMU reading library
#include <SparkFun_BMI270_Arduino_Library.h>

// PPM signal output library
#include <PWMServo.h>

// GPS encode library
#include <TinyGPSPlus.h>

// ********************** Project Objects *********************

// The TinyGPSPlus object
TinyGPSPlus gps;

// Display variables
const uint8_t DISPLAY_WIDTH = 128;
const uint8_t DISPLAY_HEIGHT = 64;
const uint8_t DISPLAY_ADDRESS = 0x3C;
const int8_t DISPLAY_RESET = -1;

// Display object
Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, DISPLAY_RESET);

// IMU object
BMI270 imu;

// I2C address selection
const uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68

// Radio control object
RF24 radio(9, 10); // 1st -> CE | 2nd -> CSN

// ********************** Project Variables *******************
// Radio variables
const uint8_t radio_CHANNEL = 71;
const uint8_t radio_ADDRESSES[][6] = {{0xB6, 0xE9, 0x00, 0x7A, 0x43}, {0x10, 0xCA, 0x87, 0x3F, 0xD8}};
const bool radioNumber = 1;

// Controller variables structure
typedef struct
{
  bool buttonMCL_reading = true;
  bool buttonMEL_reading = true;
  bool buttonMER_reading = true;
  bool buttonMCR_reading = true;
  bool buttonEL_reading = true;
  bool buttonER_reading = true;
  bool buttonJL_reading = true;
  bool buttonJR_reading = true;
  uint16_t XLaxis_reading = 0;
  uint16_t YLaxis_reading = 0;
  uint16_t XRaxis_reading = 0;
  uint16_t YRaxis_reading = 0;
  uint16_t sliderL_reading = 0;
  uint16_t sliderR_reading = 0;
  uint16_t rotaryL_reading = 0;
  uint16_t rotaryR_reading = 0;
  int8_t encoderL_reading = 0;
  int8_t encoderR_reading = 0;
} controller_variables;
controller_variables controller;

// Message receiving variables
uint8_t channel;
uint8_t bytes;
unsigned long last_message = 0;
const int FAILSAFE_INTERVAL = 5000;
const uint8_t LED_RCV = 32;

// Display timing variables
unsigned long last_display = 0;
const uint8_t DISPLAY_INTERVAL = 10;

// Buttons reading pins
#define BOTAO1 14
#define BOTAO2 15
#define BOTAO3 16
#define BOTAO4 17

// Buttons readings objects
Bounce botao1 = Bounce();
Bounce botao2 = Bounce();
Bounce botao3 = Bounce();
Bounce botao4 = Bounce();

// Buttons readings variables
const int DEBOUNCE_INTERVAL = 1000;
bool display_controller = false;
bool display_imu = false;
bool display_distance = false;
bool enable_output = false;

// Output pins
const uint8_t PWM1_OUT = 8;
const uint8_t PWM2_OUT = 7;
const uint8_t PWM3_OUT = 6;
const uint8_t PWM4_OUT = 5;
const uint8_t PWM5_OUT = 4;
const uint8_t PWM6_OUT = 3;
const uint8_t PWM7_OUT = 20;
const uint8_t PWM8_OUT = 21;
const uint8_t PPM1_OUT = 22;
const uint8_t PPM2_OUT = 23;
const uint8_t DIG1_OUT = 29;
const uint8_t DIG2_OUT = 30;
const uint8_t DIG3_OUT = 31;
const uint8_t DIG4_OUT = 28;
const uint8_t DIG5_OUT = 27;
const uint8_t DIG6_OUT = 26;
const uint8_t DIG7_OUT = 25;
const uint8_t DIG8_OUT = 24;
const uint8_t LED_OUT = 33;

// PPM output objects
PWMServo PPM1;
PWMServo PPM2;

// Distance reference variables
double reference_lng = 0;
double reference_lat = 0;
float distance = 0;

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
void displayController()
{
  display.clearDisplay();

  drawJoyPosition(22, 22, 22, map(controller.XLaxis_reading, 0, 4095, 2, 42), map(controller.YLaxis_reading, 0, 4095, 2, 42));
  drawPottentiometerBar(46, 0, 6, 44, map(controller.rotaryL_reading, 0, 4095, 0, 100));
  drawPottentiometerBar(54, 0, 6, 44, map(controller.sliderL_reading, 0, 4095, 100, 0));
  drawPottentiometerBar(61, 0, 6, 22, map(controller.encoderL_reading, -90, 90, 0, 100));
  drawPottentiometerBar(61, 22, 6, 22, map(controller.encoderR_reading, -90, 90, 0, 100));
  drawPottentiometerBar(68, 0, 6, 44, map(controller.sliderR_reading, 0, 4095, 100, 0));
  drawPottentiometerBar(76, 0, 6, 44, map(controller.rotaryR_reading, 0, 4095, 0, 100));
  drawJoyPosition(105, 22, 22, map(controller.XRaxis_reading, 0, 4095, 86, 126), map(controller.YRaxis_reading, 0, 4095, 2, 42));

  drawButtonState(0, 46, 32, 8, !controller.buttonJL_reading);
  drawButtonState(32, 46, 32, 8, !controller.buttonEL_reading);
  drawButtonState(64, 46, 32, 8, !controller.buttonER_reading);
  drawButtonState(96, 46, 32, 8, !controller.buttonJR_reading);
  drawButtonState(0, 56, 32, 8, !controller.buttonMEL_reading);
  drawButtonState(32, 56, 32, 8, !controller.buttonMCL_reading);
  drawButtonState(64, 56, 32, 8, !controller.buttonMCR_reading);
  drawButtonState(96, 56, 32, 8, !controller.buttonMER_reading);

  display.display();
}

// Function to display th IMU orientation
void drawAccelPosition(int x, int y, int radius, int px, int py, bool inverted)
{
  display.drawLine(x, (y + radius) - 4, x, (y - radius) + 4, WHITE);
  display.drawLine((x + radius) - 4, y, (x - radius) + 4, y, WHITE);
  display.drawRoundRect(x - radius, y - radius, (radius * 2), (radius * 2), 5, WHITE);
  display.drawCircle(x, y, radius, WHITE);
  display.drawCircle(px, py, radius / 6, WHITE);
  if (inverted)
  {
    display.fillCircle(px, py, radius / 10, WHITE);
  }
}

// Function to read IMU and update the display
void displayIMU()
{
  display.clearDisplay();

  imu.getSensorData();

  int posicao_eixoX = map(imu.data.accelX, 1, -1, 2, 62);
  int posicao_eixoY = map(imu.data.accelY, -1, 1, 34, 94);
  bool inverted;
  if (imu.data.accelZ > 0.6)
  {
    inverted = true;
  }
  else
  {
    inverted = false;
  }

  drawAccelPosition(64, 32, 32, posicao_eixoY, posicao_eixoX, inverted);

  display.display();
}

// Function to calculate the distance and display it
void displayDistance()
{
  while (Serial1.available() > 0)
  {
    gps.encode(Serial1.read());
    if (gps.location.isUpdated())
    {
      distance = gps.distanceBetween(reference_lat, reference_lng, gps.location.lat(), gps.location.lng());
      Serial.println(distance);

      display.clearDisplay();

      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println(F("DISTANCE (m):"));

      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 10);
      display.println(distance);

      display.display();
    }
  }
}

// ********************** Project Configuration ***************
void setup()
{

  // Radio initialization
  Serial.begin(9600);
  Serial1.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    while (1)
    {
    }
  }
  display.clearDisplay();
  display.display();

  // IMU initialization
  while (imu.beginI2C(i2cAddress) != BMI2_OK)
  {
    // Not connected, inform user
    Serial.println("Error: BMI270 not connected, check wiring and I2C address!");

    // Wait a bit to see if connection is established
    delay(1000);
  }

  // Output pins initialization
  pinMode(PWM1_OUT, OUTPUT);
  pinMode(PWM2_OUT, OUTPUT);
  pinMode(PWM3_OUT, OUTPUT);
  pinMode(PWM4_OUT, OUTPUT);
  pinMode(PWM5_OUT, OUTPUT);
  pinMode(PWM6_OUT, OUTPUT);
  pinMode(PWM7_OUT, OUTPUT);
  pinMode(PWM8_OUT, OUTPUT);
  analogWrite(PWM1_OUT, 0);
  analogWrite(PWM2_OUT, 0);
  analogWrite(PWM3_OUT, 0);
  analogWrite(PWM4_OUT, 0);
  analogWrite(PWM5_OUT, 0);
  analogWrite(PWM6_OUT, 0);
  analogWrite(PWM7_OUT, 0);
  analogWrite(PWM8_OUT, 0);

  PPM1.attach(PWM1_OUT);
  PPM2.attach(PWM2_OUT);
  PPM1.write(90);
  PPM2.write(90);

  pinMode(DIG1_OUT, OUTPUT);
  pinMode(DIG2_OUT, OUTPUT);
  pinMode(DIG3_OUT, OUTPUT);
  pinMode(DIG4_OUT, OUTPUT);
  pinMode(DIG5_OUT, OUTPUT);
  pinMode(DIG6_OUT, OUTPUT);
  pinMode(DIG7_OUT, OUTPUT);
  pinMode(DIG8_OUT, OUTPUT);
  digitalWrite(DIG1_OUT, LOW);
  digitalWrite(DIG2_OUT, LOW);
  digitalWrite(DIG3_OUT, LOW);
  digitalWrite(DIG4_OUT, LOW);
  digitalWrite(DIG5_OUT, LOW);
  digitalWrite(DIG6_OUT, LOW);
  digitalWrite(DIG7_OUT, LOW);
  digitalWrite(DIG8_OUT, LOW);

  pinMode(LED_OUT, OUTPUT);
  digitalWrite(LED_OUT, LOW);

  // Receiver radio initialization
  if (!radio.begin())
  {
    Serial.println(F("Falha na inicializacao do radio"));
    while (1)
    {
    }
  }
  Serial.println("Radio inicializado!");

  // Radio configuration
  radio.setChannel(radio_CHANNEL);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(sizeof(controller));
  radio.openWritingPipe(radio_ADDRESSES[radioNumber]);
  radio.openReadingPipe(1, radio_ADDRESSES[!radioNumber]);
  radio.startListening();
  pinMode(LED_RCV, OUTPUT);
  digitalWrite(LED_RCV, LOW);

  // Buttons configurations
  botao1.attach(BOTAO1, INPUT);
  botao2.attach(BOTAO2, INPUT);
  botao3.attach(BOTAO3, INPUT);
  botao4.attach(BOTAO4, INPUT);

  botao1.interval(DEBOUNCE_INTERVAL);
  botao2.interval(DEBOUNCE_INTERVAL);
  botao3.interval(DEBOUNCE_INTERVAL);
  botao4.interval(DEBOUNCE_INTERVAL);
}

// ********************** Project Main Loop ***************
void loop()
{

  // Checks if radio is in range, and reads it
  if (radio.available(&channel))
  {
    bytes = radio.getPayloadSize();
    digitalWrite(LED_RCV, HIGH);
    last_message = millis();
  }
  // Otherwise, enable failsafe to outputs
  else if ((millis() - last_message) > FAILSAFE_INTERVAL)
  {
    digitalWrite(LED_RCV, LOW);
    if (enable_output)
    {
      analogWrite(PWM1_OUT, 0);
      analogWrite(PWM2_OUT, 0);
      analogWrite(PWM3_OUT, 0);
      analogWrite(PWM4_OUT, 0);
      analogWrite(PWM5_OUT, 0);
      analogWrite(PWM6_OUT, 0);
      analogWrite(PWM7_OUT, 0);
      analogWrite(PWM8_OUT, 0);
      PPM1.write(90);
      PPM2.write(90);
      digitalWrite(DIG1_OUT, LOW);
      digitalWrite(DIG2_OUT, LOW);
      digitalWrite(DIG3_OUT, LOW);
      digitalWrite(DIG4_OUT, LOW);
      digitalWrite(DIG5_OUT, LOW);
      digitalWrite(DIG6_OUT, LOW);
      digitalWrite(DIG7_OUT, LOW);
      digitalWrite(DIG8_OUT, LOW);
    }
  }

  // Checks if button1 is pressed
  botao1.update();
  if (botao1.changed())
  {
    if (botao1.read() == LOW)
    {
      display_distance = !display_distance;
      if (display_controller || display_imu)
      {
        display_controller = false;
        display_imu = false;
      }
      if (display_distance)
      {
        while (Serial1.available() > 0)
        {
          gps.encode(Serial1.read());
          reference_lng = gps.location.lng();
          reference_lat = gps.location.lat();
          Serial.print("Latitude= ");
          Serial.print(reference_lat, 6);
          Serial.print(" Longitude= ");
          Serial.println(reference_lng, 6);
        }
      }
    }
  }

  // Checks if button2 is pressed
  botao2.update();
  if (botao2.changed())
  {
    if (botao2.read() == LOW)
    {
      display_controller = !display_controller;
      if (display_distance || display_imu)
      {
        display_distance = false;
        display_imu = false;
      }
    }
  }

  // Checks if button3 is pressed
  botao3.update();
  if (botao3.changed())
  {
    if (botao3.read() == LOW)
    {
      display_imu = !display_imu;
      if (display_distance || display_controller)
      {
        display_distance = false;
        display_controller = false;
      }
    }
  }

  // Checks if button4 is pressed
  botao4.update();
  if (botao4.changed())
  {
    if (botao4.read() == LOW)
    {
      enable_output = !enable_output;
    }
  }

  // If output is enabled
  if (enable_output)
  {
    digitalWrite(LED_OUT, HIGH);
    analogWrite(PWM1_OUT, map(controller.XLaxis_reading, 0, 4095, 0, 255));
    analogWrite(PWM2_OUT, map(controller.YLaxis_reading, 0, 4095, 0, 255));
    analogWrite(PWM3_OUT, map(controller.XRaxis_reading, 0, 4095, 0, 255));
    analogWrite(PWM4_OUT, map(controller.YRaxis_reading, 0, 4095, 0, 255));
    analogWrite(PWM5_OUT, map(controller.rotaryL_reading, 0, 4095, 0, 255));
    analogWrite(PWM6_OUT, map(controller.rotaryR_reading, 0, 4095, 0, 255));
    analogWrite(PWM7_OUT, map(controller.sliderL_reading, 0, 4095, 0, 255));
    analogWrite(PWM8_OUT, map(controller.sliderR_reading, 0, 4095, 0, 255));
    PPM1.write(map(controller.encoderL_reading, -180, 180, 0, 180));
    PPM2.write(map(controller.encoderR_reading, -180, 180, 0, 180));
    digitalWrite(DIG1_OUT, !controller.buttonJL_reading);
    digitalWrite(DIG2_OUT, !controller.buttonJR_reading);
    digitalWrite(DIG3_OUT, !controller.buttonEL_reading);
    digitalWrite(DIG4_OUT, !controller.buttonER_reading);
    digitalWrite(DIG5_OUT, !controller.buttonMEL_reading);
    digitalWrite(DIG6_OUT, !controller.buttonMCL_reading);
    digitalWrite(DIG7_OUT, !controller.buttonMER_reading);
    digitalWrite(DIG8_OUT, !controller.buttonMCR_reading);
  }
  else
  {
    digitalWrite(LED_OUT, LOW);
    analogWrite(PWM1_OUT, 0);
    analogWrite(PWM2_OUT, 0);
    analogWrite(PWM3_OUT, 0);
    analogWrite(PWM4_OUT, 0);
    analogWrite(PWM5_OUT, 0);
    analogWrite(PWM6_OUT, 0);
    analogWrite(PWM7_OUT, 0);
    analogWrite(PWM8_OUT, 0);
    PPM1.write(90);
    PPM2.write(90);
    digitalWrite(DIG1_OUT, LOW);
    digitalWrite(DIG2_OUT, LOW);
    digitalWrite(DIG3_OUT, LOW);
    digitalWrite(DIG4_OUT, LOW);
    digitalWrite(DIG5_OUT, LOW);
    digitalWrite(DIG6_OUT, LOW);
    digitalWrite(DIG7_OUT, LOW);
    digitalWrite(DIG8_OUT, LOW);
  }

  // If display exibition is enabled
  if ((millis() - last_display) > DISPLAY_INTERVAL)
  {
    if (display_controller)
    {
      displayController();
    }
    else if (display_imu)
    {
      displayIMU();
    }
    else if (display_distance)
    {
      displayDistance();
    }
    else
    {
      display.clearDisplay();
      display.display();
    }
    last_display = millis();
  }
}
// ************************************************************
