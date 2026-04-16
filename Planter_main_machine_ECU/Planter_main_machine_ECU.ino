

char arduinoDate[] = "2026-04-11";
char firmwareName[] = "JD1770NT main machine ECU";
char arduinoVersion[] = "v 1.0.7";
/*
Teensy Pinout
GND
0 (ECU pin 25-53 IN) RX1 SerialPop
1 (ECU pin 26-54 OUT) TX1 SerialPop
2 (ECU pin 36)Pin9 digital input from PWR circuit
3 (ECU pin 1)Pin1 output
4 (ECU pin 29)Pin2 output
5 (ECU pin 30)Pin3 output
6 (ECU pin 31)Pin4 output
7 (ECU pin 32)Pin5 output
8 (ECU pin 33)Pin6 output
9 (ECU pin 34)Pin7 output
10 (ECU pin 35)Pin8 output
11 (ECU pin 48) digital input3
12 (ECU pin 47) digital input4
3.3V
24
25
26
27
28 (ECU pin 49) digital input2
29 (ECU pin 50) digital input1
30 (ECU pin 27-55 CANL) rxCAN AiO
31 (ECU pin 28-56 CANH) txCAN AiO
32
---------
Vin
GND
3.3V
23 A9 (ECU pin 14) Pin16
22 A8 (ECU pin 42) Pin15
21
20
19 A5 (ECU pin 41) Pin14
18 A4 (ECU pin 40) Pin13
17 (ECU pin 51 A) TX4 RS485-2 unused
16 (ECU pin 23 B) RX4 RS485-2 unused
15
14
13
GND
41 A17 (ECU pin 19) analog input4
40 A16 (ECU pin 20) analog input3
39 A15 (ECU pin 21) analog input2
38 A14 (ECU pin 22) analog input1
37 (ECU pin 1) Pin12 digital input from PWR circuit
36 (ECU pin 1) Pin11 digital input from PWR circuit
35 (ECU pin 52 A) TX8 RS485-1 downforce
34 (ECU pin 24 B) RX8 RS485-1 downforce
33 (ECU pin 1) Pin10 digital input from PWR circuit
*/

/*  PWM Frequency -> 
   *   490hz (default) = 0
   *   122hz = 1
   *   3921hz = 2
   */
#define PWM_Frequency 1

//Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency);
extern float tempmonGetTemp(void);

//loop time variables in milliseconds
const uint8_t LOOP_TIME = 100;  // 10Hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

//EEPROM
#include <EEPROM.h>
#define EEP_Ident 0x5422
int16_t EEread = 0;
struct __attribute__((packed)) Storage {
  uint16_t heightDown = 55;
  uint16_t heightUp = 1600;
  uint8_t OnThreshold = 50;
  uint8_t OffThreshold = 100;
  uint16_t vaccum1zero = 55;
  uint16_t vaccum2zero = 55;
  int16_t vaccum1multi = 1000;
  int16_t vaccum2multi = 1000;
  uint16_t downforce1zero = 55;
  int16_t downforce1multi = 1000;
  uint16_t downforce2zero = 55;
  int16_t downforce2multi = 1000;
  uint16_t downforce3zero = 55;
  int16_t downforce3multi = 1000;
  uint16_t downPressureZero = 55;
  int16_t downPressureMulti = 1000;
};
Storage settings;  //30 bytes

//communication
uint8_t CANreceiveBuffer[16][288];
uint8_t globalBuffer[288];
uint8_t AOGtoCAN[288] = { 0 };  // Forces all elements to 0
uint8_t AOGtoCANseq = 0;
void EncodeAOGtoCAN(const uint8_t* data, uint8_t dataLen, bool isSentToAOG = true);  //to make the compiler happy, probably because of the optional argument

///////main for the pop serial reading/////////////////////////////////////////////////////////
#define SerialPop Serial1
uint8_t popRxBuffer[2048];
uint8_t popTxBuffer[2048];
uint32_t bautPop = 460800;
//Parsing PGN
bool isHeaderFound = false;
uint16_t tempHeader = 0;
bool isLengthFound = false;
uint16_t header = 0;
uint16_t temp = 0;
uint8_t serialSource = 0;
uint8_t serialPgn = 0;
uint8_t serialLength = 0;
uint8_t serialData[256];  //just to be sure it's long enough
uint8_t serialCRC = 0;
////////////////////////////////////////////////////////////////////////////////////////////


//define inputs and outputs
//outputs
//#define PWM1_CYTRON 3
//#define DIR1_CYTRON 4

//digital inputs
//#define BOUTON_UP 6
//#define BOUTON_DOWN 7

//analog inputs
#define HEIGHT_SENSOR A14
//#define POTO_UP A0
//#define POTO_DOWN A1

//input/output variables
uint16_t heightRaw = 0;
uint8_t heightPlanter;
uint8_t onThreshold = 50;
uint8_t offThreshold = 100;
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
  SerialPop.begin(bautPop);
  SerialPop.addMemoryForRead(popRxBuffer, sizeof(popRxBuffer));
  SerialPop.addMemoryForWrite(popTxBuffer, sizeof(popTxBuffer));

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
  //pinMode(BOUTON_UP, INPUT);  //INSTEAD INPUT_PULLUP, not needed?
  //pinMode(BOUTON_DOWN, INPUT);
  //pinMode(DIR1_CYTRON, OUTPUT);

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

    CanCheckOldArray();
    //analogRead(BOUTON_UP);
    //analogWrite(PWM1_CYTRON, 128);
    heightRaw = analogRead(HEIGHT_SENSOR);
    int32_t tempHeight = map(heightRaw, settings.heightDown, settings.heightUp, 1, 255);
    heightPlanter = constrain(tempHeight, 1, 255);
    Serial.print(heightPlanter);
    Serial.print(", ");
    Serial.println(heightRaw);




    //Send the Height PGN
    AOGtoCAN[0] = 0x80;
    AOGtoCAN[1] = 0x81;
    AOGtoCAN[2] = 0x7B;  //Source
    AOGtoCAN[3] = 0xA0;  //PGN
    AOGtoCAN[4] = 8;     //lenght
    AOGtoCAN[5] = highByte(heightRaw);
    AOGtoCAN[6] = lowByte(heightRaw);
    AOGtoCAN[7] = heightPlanter;
    // no AOGtoCAN[8]
    AOGtoCAN[9] = onThreshold;
    AOGtoCAN[10] = offThreshold;
    // no AOGtoCAN[11]
    // no AOGtoCAN[12]
    //do CRC
    uint8_t crc = calculateCRC(AOGtoCAN, 13);
    AOGtoCAN[13] = crc;
    SerialPop.write(AOGtoCAN, 14);
    EncodeAOGtoCAN(AOGtoCAN, 14);
    memset(AOGtoCAN, 0, 14);

  }  // end of 100 ms loop


  CanDecode();
  //to add: read the CANreceiveBuffer
  CheckDataFromCAN();

  //This runs continuously, not timed //// Serial Receive Data/Settings /////////////////
  // if there's data available, read a packet

  if (SerialPop.available() > 0 && !isHeaderFound) {
    temp = SerialPop.read();
    header = tempHeader << 8 | temp;            //high,low bytes to make int
    tempHeader = temp;                          //save for next time
    if (header == 32897) isHeaderFound = true;  //Do we have a match?
  }

  if (isHeaderFound && !isLengthFound && SerialPop.available() > 2) {
    serialSource = SerialPop.read();
    serialPgn = SerialPop.read();
    serialLength = SerialPop.read();
    if (serialLength > 0) isLengthFound = true;
    else isHeaderFound = false;  //corupt data
  }

  if (isLengthFound && SerialPop.available() > serialLength) {
    //We have all data, reset for next time
    isHeaderFound = false;
    isLengthFound = false;

    //send to CAN3
    AOGtoCAN[0] = 0x80;
    AOGtoCAN[1] = 0x81;
    AOGtoCAN[2] = serialSource;
    AOGtoCAN[3] = serialPgn;
    AOGtoCAN[4] = serialLength;
    SerialPop.readBytes(&AOGtoCAN[5], serialLength + 1);
    //todo: check CRC, if bad, return, if good continue

    EncodeAOGtoCAN(AOGtoCAN, serialLength + 6);
    memset(AOGtoCAN, 0, serialLength + 6);
  }
  /////////end of serial to CAN//////////////////////
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


        uint8_t crc = calculateCRC(globalBuffer, 5 + dataLen);
        globalBuffer[5 + dataLen] = crc;

        if (dataSrc == 123 || (dataSrc == 127 && dataPGN == 239)) {
          SerialPop.write(globalBuffer, 6 + dataLen);
        }
        //other here
        if (dataSrc == 123 && dataPGN == 161) {
          //7B A1 height config
          uint16_t temp = 0;
          temp = ((uint16_t)globalBuffer[5] << 8) | (uint8_t)globalBuffer[6];
          if (temp < 4096) settings.heightDown = temp;
          temp = ((uint16_t)globalBuffer[7] << 8) | (uint8_t)globalBuffer[8];
          if (temp < 4096) settings.heightUp = temp;
          temp = (uint8_t)globalBuffer[9];
          if (temp < 255) settings.OnThreshold = temp;
          temp = (uint8_t)globalBuffer[10];
          if (temp < 255) settings.OffThreshold = temp;

          EEPROM.put(6, settings);
        }
      }
    }
  }
}

// Calculate CRC for PGN message
uint8_t calculateCRC(uint8_t* buffer, uint8_t length) {
  uint8_t crc = 0;
  for (int i = 2; i < length; i++) {
    crc += buffer[i];
  }
  return crc;
}