/**************************************************************
 * Everything Controller (v2.0) - Receiver Example
 *
 * Base code for the "Everything Controller V2.0" receiver.
 * Communication is based on RF24 using the NRF24L01+ module. The controller
 * sends data from 8 digital buttons, 2 joysticks (2 X-axis and 2 Y-axis), 
 * two sliders, two rotary potentiometers and two rotary encoders. 
 * With this data, it is possible to control anything.
 *
 * Written by Giovanni de Castro (10/03/2024).
 *************************************************************/

// Redefinition of the SPI speed for faster communication
#define RF24_SPI_SPEED 16000000

// Libraries
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

// Radio control object
RF24 radio(9, 10);  // 1st -> CE | 2nd -> CSN

// Radio variables
const uint8_t radio_CHANNEL = 71;
const uint8_t radio_ADDRESSES[][6] = { { 0xB6, 0xE9, 0x00, 0x7A, 0x43 }, { 0x10, 0xCA, 0x87, 0x3F, 0xD8 } };
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
long last_message = 0;
unsigned long last_rx = 0;
const uint8_t RX_INTERVAL = 2;

void setup() {

  // Radio initialization
  Serial.begin(9600);
  if (!radio.begin()) {
    Serial.println(F("Falha na inicializacao do radio"));
    while (1) {}
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
  last_message = millis();
}

// Repeticao do codigo
void loop() {

  if ((millis() - last_rx) > RX_INTERVAL) {
    if (radio.available(&channel)) {
      bytes = radio.getPayloadSize();
      radio.read(&controller, bytes);
      Serial.print(F("Mensagem de "));
      Serial.print(bytes);
      Serial.print(F(" bytes recebida no channel "));
      Serial.print(channel);
      Serial.print(F(" | esperado : "));
      Serial.print(sizeof(controller));
      Serial.print(F(" bytes em : "));
      Serial.print(millis() - last_message);
      Serial.println(F(" ms"));
      last_message = millis();
    }
  }
}
