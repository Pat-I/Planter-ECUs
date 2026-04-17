

char arduinoDate[] = "2026-04-15";
char firmwareName[] = "JD1770NT fertilizer ECU";
char arduinoVersion[] = "v 1.0.0";

bool engageFromHigh = true;
uint8_t stopTime = 100;                    // time the solenoids remain powered if stopped, secX10, 100 is 10 sec, max 200(20sec)
uint8_t solenoidActivationDelayTime = 50;  // time the solenoids will not be powered when lowerded, secX10, 100 is 10 sec, max 200(20sec)
/*
Teensy Pinout
GND
0 (ECU pin 25-53 IN) not used
1 (ECU pin 26-54 OUT) not used
2 (ECU pin 36)Pin9 input
3 (ECU pin 1)Pin1 output
4 (ECU pin 29)Pin2 output
5 (ECU pin 30)Pin3 output
6 (ECU pin 31)Pin4 output
7 (ECU pin 32)Pin5 output
8 (ECU pin 33)Pin6 output
9 (ECU pin 34)Pin7 output
10 (ECU pin 35)Pin8 output
11 (ECU pin 48) digital input3 not used
12 (ECU pin 47) digital input4 not used
3.3V
24
25
26
27
28 (ECU pin 49) digital input2 not used
29 (ECU pin 50) digital input1 not used
30 (ECU pin 27-55 CANL) rxCAN AiO
31 (ECU pin 28-56 CANH) txCAN AiO
32
---------
Vin
GND
3.3V
23 A9 (ECU pin 14) Pin16 input
22 A8 (ECU pin 42) Pin15 input
21
20
19 A5 (ECU pin 41) Pin14 input
18 A4 (ECU pin 40) Pin13 input
17 (ECU pin 51 A) TX4 RS485-2 unused
16 (ECU pin 23 B) RX4 RS485-2 unused
15
14
13
GND
41 A17 (ECU pin 19) analog input4 not used
40 A16 (ECU pin 20) analog input3 not used
39 A15 (ECU pin 21) analog input2 not used
38 A14 (ECU pin 22) analog input1 not used
37 (ECU pin 1) Pin12 input
36 (ECU pin 1) Pin11 input
35 (ECU pin 52 A) TX8 RS485 fertilizer
34 (ECU pin 24 B) RX8 Rs485 fertilizer
33 (ECU pin 1) Pin10 input
*/
uint8_t solenoid[] = { 3, 4, 5, 6, 7, 8, 9, 10 };
uint8_t hall[] = { 2, 33, 36, 37, 18, 19, 22, 23 };
uint8_t forceOnTime[8];

/*  PWM Frequency -> 
   *   490hz (default) = 0
   *   122hz = 1
   *   3921hz = 2
   */
#define PWM_Frequency 1

//EEPROM
#include <EEPROM.h>
#define EEP_Ident 0x5422
int16_t EEread = 0;
struct __attribute__((packed)) Storage {
  int16_t fertilizerZero = 55;
};
Storage settings;  //30 bytes

//Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency);
extern float tempmonGetTemp(void);

//loop time variables in milliseconds
const uint8_t LOOP_TIME = 100;  // 10Hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

//communication
uint8_t CANreceiveBuffer[16][288];
uint8_t globalBuffer[288];
uint8_t AOGtoCAN[288] = { 0 };  // Forces all elements to 0
uint8_t AOGtoCANseq = 0;
void EncodeAOGtoCAN(const uint8_t* data, uint8_t dataLen, bool isSentToAOG = true);  //to make the compiler happy, probably because of the optional argument

///////main for the pop serial reading/////////////////////////////////////////////////////////
#define SerialRS485 Serial8
uint8_t rs485RxBuffer[2048];
uint8_t rs485TxBuffer[2048];
uint32_t bautRS485 = 9600;
//Parsing PGN
bool isRS485HeaderFound = false;
uint16_t RS485tempHeader = 0;
uint16_t RS485header = 0;
uint16_t RS485temp = 0;

//input/output variables
bool isPlanterLowered = true;
uint8_t numPlanterRows = 16;
uint8_t numSolenoid = 8;
uint8_t heightPlanter = 0;
uint8_t onThreshold = 50;
uint8_t offThreshold = 100;
uint8_t AOGSpeedX10 = 0;
uint8_t millisSectionStatus = 0;
bool isSolenoidActive[8] = { 0 };
bool isTrapOpen[8] = { 0 };
uint8_t rowSectionStatus[2] = { 0 };
bool fertilizerSectionStatus[8] = { 0 };  //true is fertilizing

uint8_t speedTimer = 0;
uint8_t solenoidActivationTimer[8] = { 0 };
int16_t weightActual = 0;  //in kg
int32_t weightRaw = 0;
/////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Core at 150 MHz (To reduce heat)
  set_arm_clock(150000000);
  //PWM rate settings. Set them both the same!!!!
  /*  PWM Frequency ->
       490hz (default) = 0
       122hz = 1
       3921hz = 2
  */
  if (PWM_Frequency == 0) {
    //analogWriteFrequency(PWM1_LPWM, 490);
    //analogWriteFrequency(PWM2_RPWM, 490);
  } else if (PWM_Frequency == 1) {
    //analogWriteFrequency(PWM1_LPWM, 122);
    //analogWriteFrequency(PWM2_RPWM, 122);
  } else if (PWM_Frequency == 2) {
    //analogWriteFrequency(PWM1_LPWM, 3921);
    //analogWriteFrequency(PWM2_RPWM, 3921);
  }
  Serial.begin(115200);
  SerialRS485.begin(bautRS485);
  SerialRS485.addMemoryForRead(rs485RxBuffer, sizeof(rs485RxBuffer));
  SerialRS485.addMemoryForWrite(rs485TxBuffer, sizeof(rs485TxBuffer));

  analogReadResolution(12);  //read 0-4095 on analog pins
  analogReadAveraging(8);    //takes 15us
  //set the inputs independantly from the pin names
  pinMode(A14, INPUT_DISABLE);  //analog input1
  pinMode(A15, INPUT_DISABLE);  //analog input2
  pinMode(A16, INPUT_DISABLE);  //analog input3
  pinMode(A17, INPUT_DISABLE);  //analog input4
  pinMode(29, INPUT_PULLUP);    //digital1
  pinMode(28, INPUT_PULLUP);    //digital2
  pinMode(11, INPUT_PULLUP);    //digital3
  pinMode(12, INPUT_PULLUP);    //digital4

  //pinMode is only for digital pins?
  for (uint8_t i = 0; i < numSolenoid; i++) {
    pinMode(solenoid[i], OUTPUT);
    pinMode(hall[i], INPUT_PULLUP);
  }

  //EEPROM
  EEPROM.get(0, EEread);  // read identifier

  if (EEread != EEP_Ident)  // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(6, settings);  //Machine
  } else {
    EEPROM.get(6, settings);  //Machine
  }

  delay(100);
  Serial.println(firmwareName);
  Serial.println(arduinoVersion);
  Serial.println(arduinoDate);

  Caninit();
}

void loop() {
  // put your main code here, to run repeatedly:

  //Loop triggers every 100 msec
  currentTime = millis();

  if (currentTime - lastTime >= LOOP_TIME) {
    lastTime = currentTime;
    millisSectionStatus++;

    if (isPlanterLowered) {
      for (int j = 0; j < 8; j++) {
        if (solenoidActivationTimer[j] > 0) solenoidActivationTimer[j]--;
      }
    } else memset(solenoidActivationTimer, solenoidActivationDelayTime, numSolenoid);  //solenoidActivationTimer = solenoidActivationDelayTime;

    if (AOGSpeedX10 > 2) speedTimer = 0;
    else if (speedTimer < 250) speedTimer++;
    // check if a field is connected
    if (millisSectionStatus > 3) {
      millisSectionStatus = 0;  // wait 0.4 sec to do it again if still no connection

      memset(isSolenoidActive, 0, numSolenoid);
      SetSolenoids();
      AOGSpeedX10 = 0;
    }

    //send the fertilizer PGN 7B A6
    //check trap position, reading HIGH is trap closed, LOW is trap open
    for (uint8_t i = 0; i < numSolenoid; i++) {
      isTrapOpen[i] = !digitalRead(hall[i]);
    }
    uint8_t setPos = 0;
    uint8_t actPos = 0;
    for (int i = 0; i < 8; i++) {
      if (!isSolenoidActive[i]) {
        setPos |= (1 << i);  // Set the bit at position 'i' to 1
      }
      if (isTrapOpen[i]) {
        actPos |= (1 << i);  // Set the bit at position 'i' to 1
      }
    }
    AOGtoCAN[0] = 0x80;
    AOGtoCAN[1] = 0x81;
    AOGtoCAN[2] = 0x7B;  //Source
    AOGtoCAN[3] = 0xA6;  //PGN
    AOGtoCAN[4] = 8;     //lenght
    AOGtoCAN[5] = highByte(weightActual);
    AOGtoCAN[6] = lowByte(weightActual);
    AOGtoCAN[7] = actPos;  //1 to 8
    // no AOGtoCAN[8] //9 to 16
    AOGtoCAN[9] = setPos;  //1 to 8
    // no AOGtoCAN[10] = 0; // 9 to 16
    // no AOGtoCAN[11]
    // no AOGtoCAN[12]
    //do CRC
    uint8_t crc = calculateCRC(AOGtoCAN, 13);
    AOGtoCAN[13] = crc;
    EncodeAOGtoCAN(AOGtoCAN, 14);
    memset(AOGtoCAN, 0, 14);

    CanCheckOldArray();
  }  // end of 100 ms loop

  CanDecode();
  CheckDataFromCAN();

  //This runs continuously, not timed //// RS485 Receive Data/Settings /////////////////
  // if there's data available, read a packet

  if (SerialRS485.available() > 0 && !isRS485HeaderFound) {
    RS485temp = SerialRS485.read();
    RS485header = RS485tempHeader << 8 | RS485temp;       //high,low bytes to make int
    RS485tempHeader = RS485temp;                          //save for next time
    if (RS485header == 32897) isRS485HeaderFound = true;  //Do we have a match?
  }

  if (isRS485HeaderFound && SerialRS485.available() >= 6) {
    //We have all data, reset for next time
    isRS485HeaderFound = false;

    uint8_t id = SerialRS485.read();
    uint8_t b1 = SerialRS485.read();
    uint8_t b2 = SerialRS485.read();
    uint8_t b3 = SerialRS485.read();
    uint8_t b4 = SerialRS485.read();
    uint8_t receivedCksum = SerialRS485.read();

    // Verify Checksum
    uint8_t calculatedCksum = id + b1 + b2 + b3 + b4;

    if (calculatedCksum == receivedCksum) {
      // Success! Rebuild the long
      weightRaw = ((int32_t)b4 << 24) | ((int32_t)b3 << 16) | ((int32_t)b2 << 8) | b1;
    }
  }
  /////////end of RS485 receiving//////////////////////
}  // end of loop

void CheckDataFromCAN() {
  for (uint8_t i = 0; i < 16; i++) {
    if (CANreceiveBuffer[i][0] == 1) {
      CANreceiveBuffer[i][0] = 0;  //read and ready to be re-used

      //format:
      // code, loopCounter, sequence, Source, Dest, lenght, Data......., CRC (only if data > 8)
      uint8_t dataSrc = CANreceiveBuffer[i][3];
      uint8_t dataPGN = CANreceiveBuffer[i][4];
      uint8_t dataLen = CANreceiveBuffer[i][5];
      globalBuffer[0] = 0x80;
      globalBuffer[1] = 0x81;
      globalBuffer[2] = dataSrc;
      globalBuffer[3] = dataPGN;
      globalBuffer[4] = dataLen;

      if (dataLen > 0) {
        memcpy(&globalBuffer[5], &CANreceiveBuffer[i][6], dataLen);
      }

      uint8_t crc = calculateCRC(globalBuffer, 5 + dataLen);
      globalBuffer[5 + dataLen] = crc;

      //read the revelent PGNs

      if (dataSrc == 123)  // 7B from AOG Planter monitor
      {
        if (dataPGN == 160)  //A0 Height
        {
          //dont read 0 and 1 raw height
          heightPlanter = globalBuffer[7];
          //no 3
          onThreshold = globalBuffer[9];
          offThreshold = globalBuffer[10];
          //no 6, 7

          if (engageFromHigh && heightPlanter < offThreshold) isPlanterLowered = true;  //enable the solenoid from the higher point
          else if (heightPlanter < onThreshold) isPlanterLowered = true;                //enable the solenoid from the lower point
          if (heightPlanter > offThreshold) isPlanterLowered = false;
        }
        if (dataPGN == 167)  //A7 fertilizer config
        {
          int16_t tempInt = 0;
          tempInt = ((int16_t)globalBuffer[5] << 8) | (uint8_t)globalBuffer[6];
          if (tempInt < 32000) {
            settings.fertilizerZero = tempInt;
            EEPROM.put(6, settings);
          }
          uint16_t temp = 0;
          temp = ((uint16_t)globalBuffer[7] << 8) | (uint8_t)globalBuffer[8];
          if (temp < 32000) SetfertilizerScale(temp);
          temp = (uint8_t)globalBuffer[9];
          if (temp > 0) ForceStectionOn(temp);
        }
      }

      if (dataSrc == 127)  //Data from AOG
      {
        if (dataPGN == 239)  //FE autoSteerData
        {
          AOGSpeedX10 = globalBuffer[6];
          //Serial.print("Speed= ");
          //Serial.println(AOGSpeedX10);

          rowSectionStatus[0] = globalBuffer[11];
          rowSectionStatus[1] = globalBuffer[12];
          millisSectionStatus = 0;
          CheckRowStatus();
        }
      }
    }
  }
}

void CheckRowStatus() {
  if (isPlanterLowered && speedTimer < stopTime) {
    //check if section is on
    for (uint8_t i = 0; i < numSolenoid; i++) {
      if (solenoidActivationTimer[i] < 2) {
        uint8_t byteIdx = i / 4;
        uint8_t bitPos = (i % 4) * 2;

        // Isolate the 2 bits for this solenoid (masking with 0b11 which is decimal 3)
        uint8_t twoBits = (rowSectionStatus[byteIdx] >> bitPos) & 0x03;

        // isSolenoidActive is TRUE only if twoBits is 0 (both bits are 0)
        isSolenoidActive[i] = (twoBits == 0);

        // Note: If you meant it's ACTIVE if ANY bit is set, use (twoBits != 0)
      }
    }
  } else {
    //raised or stationary for a while, un-energize the cluches
    memset(isSolenoidActive, 0, numSolenoid);
  }

  SetSolenoids();
}

void SetSolenoids() {
  for (uint8_t i = 0; i < numSolenoid; i++) {
    digitalWrite(solenoid[i], isSolenoidActive[i]);
  }
}

void ForceStectionOn(uint16_t temp) {
  for (int j = 0; j < 8; j++) {
    if (bitRead(temp, j)) {
      solenoidActivationTimer[j] = solenoidActivationDelayTime;
    }
  }
}

void SetfertilizerScale(uint16_t temp) {
}
// Calculate CRC for PGN message
uint8_t calculateCRC(uint8_t* buffer, uint8_t length) {
  uint8_t crc = 0;
  for (int i = 2; i < length; i++) {
    crc += buffer[i];
  }
  return crc;
}