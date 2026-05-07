

char arduinoDate[] = "2026-05-05";
char firmwareName[] = "HX711 cell Nano for JD1770";
char arduinoVersion[] = "v 1.0.1";

/*  PWM Frequency -> 
   *   490hz (default) = 0
   *   122hz = 1
   *   3921hz = 2
   */
#define PWM_Frequency 0
#include <SoftwareSerial.h>
#include "HX711.h"

// Définition des broches : RX (6) et TX (7)
SoftwareSerial SerialRS485(6, 7);
HX711 scale1;

uint8_t dataPin1 = 4;
uint8_t clockPin1 = 5;
//uint8_t dataPin2 = 2;
//uint8_t clockPin2 = 3;


int32_t smoothWeight = 0;
const int32_t alpha = 15;  // Represents 0.15 (15%)
const int32_t beta = 85;   // Represents 0.85 (85%) - Note: alpha + beta must = 100



//loop time variables in milliseconds
const uint16_t LOOP_TIME = 200;  // 5Hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

//define inputs and outputs
//#define PWM1_CYTRON  3
//#define DIR1_CYTRON  4

//#define BOUTON_UP 6
//#define BOUTON_DOWN 7

//#define POTO_UP A0
//#define POTO_DOWN A1

void setup() {
  // put your setup code here, to run once:
  //PWM rate settings. Set them both the same!!!!

  if (PWM_Frequency == 1) {
    TCCR2B = TCCR2B & B11111000 | B00000110;  // set timer 2 to 256 for PWM frequency of   122.55 Hz
    TCCR1B = TCCR1B & B11111000 | B00000100;  // set timer 1 to 256 for PWM frequency of   122.55 Hz
  }

  else if (PWM_Frequency == 2) {
    TCCR1B = TCCR1B & B11111000 | B00000010;  // set timer 1 to 8 for PWM frequency of  3921.16 Hz
    TCCR2B = TCCR2B & B11111000 | B00000010;  // set timer 2 to 8 for PWM frequency of  3921.16 Hx
  }
  SerialRS485.begin(9600);
  Serial.begin(115200);
  //pinMode is only for digital pins?
  //pinMode(BOUTON_UP,  INPUT); //INSTEAD INPUT_PULLUP, not needed?
  //pinMode(BOUTON_DOWN, INPUT);
  //pinMode(DIR1_CYTRON,  OUTPUT);

  delay(100);

  scale1.begin(dataPin1, clockPin1);
  Serial.println(arduinoDate);
  Serial.println(firmwareName);
  Serial.println(arduinoVersion);
}

void loop() {
  // put your main code here, to run repeatedly:

  //Loop triggers every 200 msec
  currentTime = millis();
  //unsigned int time = currentTime;

  if (currentTime - lastTime >= LOOP_TIME) {
    lastTime = currentTime;
    //analogRead(BOUTON_UP);
    //analogWrite(PWM1_CYTRON, 128);



    sendPacket(1, smoothWeight);


  }  // end of 200 ms loop

  if (scale1.is_ready()) {
    // 1. Get the raw median value
    int32_t raw = scale1.read_medavg(5);  //24 bits, -8,388,608 to 8,388,607

    // 2. Initial value: If it's the first run, snap to the raw value
    if (smoothWeight == 0) {
      smoothWeight = raw;
    } else {
      // 3. Integer EMA Formula: ((New * 15) + (Old * 85)) / 100
      smoothWeight = ((raw * alpha) + (smoothWeight * beta)) / 100;
    }
  }

}  // end of loop

void sendPacket(uint8_t sensorID, int32_t weight) {
  uint8_t buf[8];
  buf[0] = 0x80;
  buf[1] = 0x81;
  buf[2] = sensorID;

  // Split long into 4 bytes (Little Endian)
  buf[3] = (weight & 0xFF);
  buf[4] = ((weight >> 8) & 0xFF);
  buf[5] = ((weight >> 16) & 0xFF);
  buf[6] = ((weight >> 24) & 0xFF);

  // Calculate Checksum (simple sum of data bytes)
  uint8_t cksum = 0;
  for (int i = 2; i < 7; i++) cksum += buf[i];
  buf[7] = cksum;

  //digitalWrite(RS485_EN, HIGH);
  SerialRS485.write(buf, 8);
  SerialRS485.flush();
  //digitalWrite(RS485_EN, LOW);
}
