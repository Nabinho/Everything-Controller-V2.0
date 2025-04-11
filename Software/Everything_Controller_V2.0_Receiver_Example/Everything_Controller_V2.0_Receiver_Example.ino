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

// Macro to disable or enable serial debugging
// #define DEBUG_ANALOG
// #define DEBUG_DIGITAL
// #define DEBUG_ENCODER

// Libraries
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

// ESP32 SPI Pins
// const uint8_t MY_MISO = 7;
// const uint8_t MY_MOSI = 6;
// const uint8_t MY_SCLK = 5;
// const uint8_t MY_SS = 4;

// Radio control object
RF24 radio(20, 19);  // 1st -> CE | 2nd -> CSN
// SPIClass *hspi = nullptr;

// Radio variables
const uint8_t radio_CHANNEL = 71;
const uint8_t radio_ADDRESSES[2][6] = {{0xB6, 0xE9, 0x00, 0x7A, 0x43}, {0x10, 0xCA, 0x87, 0x3F, 0xD8}};
const bool radioNumber = 1;

// Controller variables structure
typedef struct
{
  bool buttons_message[8];
  int analogs_message[8];
  int encoders_messsage[8];
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
  Serial.begin(115200);
  // SPI bus configuration
  // hspi = new SPIClass(HSPI);
  // hspi->begin(MY_SCLK, MY_MISO, MY_MOSI, MY_SS);
  if (!radio.begin(/*hspi*/)) {
    Serial.println(F("Falha na inicializacao do radio"));
    while (1) {
      Serial.println(F("Falha na inicializacao do radio"));
      delay(1000);
    }
  }
  Serial.println("Radio inicializado!");

  // Radio configuration
  radio.setChannel(radio_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setPayloadSize(sizeof(controller));
  radio.openWritingPipe(radio_ADDRESSES[radioNumber]);
  radio.openReadingPipe(1, radio_ADDRESSES[!radioNumber]);
  radio.startListening();
  last_message = millis();
}

// Repeticao do codigo
void loop() {

  if (radio.available(&channel)) {
    bytes = radio.getPayloadSize();
    radio.read(&controller, bytes);
    //Serial.print(F("Mensagem de "));
    //Serial.print(bytes);
    //Serial.print(F(" bytes recebida no channel "));
    //Serial.print(channel);
    //Serial.print(F(" | esperado : "));
    //Serial.print(sizeof(controller));
    //Serial.print(F(" em : "));
    Serial.print(millis() - last_message);
    Serial.println(F(" ms"));
#ifdef DEBUG_ANALOG
    for (uint8_t index = 0; index < 8; index++) {
      Serial.print(controller.analogs_message[index]);
      Serial.print(" | ");
    }
    Serial.println("");
#endif
#ifdef DEBUG_DIGITAL
    for (uint8_t index = 0; index < 8; index++) {
      Serial.print(controller.buttons_message[index]);
      Serial.print(" | ");
    }
    Serial.println("");
#endif
#ifdef DEBUG_ENCODER
    for (uint8_t index = 0; index < 2; index++) {
      Serial.print(controller.encoders_messsage[index]);
      Serial.print(" | ");
    }
    Serial.println("");
#endif
    last_message = millis();
  }
}
