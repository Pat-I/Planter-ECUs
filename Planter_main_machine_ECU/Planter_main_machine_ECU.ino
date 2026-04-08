

char arduinoDate[] = "2026-04-05";
char firmwareName[] = "JD1770NT main machine ECU";
char arduinoVersion[] = "v 1.0.6";
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

//loop time variables in milliseconds
const uint8_t LOOP_TIME = 100;  // 10Hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

//communication
uint8_t CANreceiveBuffer[16][288];
uint8_t AOGtoCAN[288] = { 0 };  // Forces all elements to 0
uint8_t AOGtoCANseq = 0;
void EncodeAOGtoCAN(const uint8_t* data, uint8_t dataLen, bool isSentToAOG = true); //to make the compiler happy, probably because of the optional argument

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
//#define PWM1_CYTRON 3
//#define DIR1_CYTRON 4

//#define BOUTON_UP 6
//#define BOUTON_DOWN 7

//#define POTO_UP A0
//#define POTO_DOWN A1

void setup() {
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
  //pinMode is only for digital pins?
  //pinMode(BOUTON_UP, INPUT);  //INSTEAD INPUT_PULLUP, not needed?
  //pinMode(BOUTON_DOWN, INPUT);
  //pinMode(DIR1_CYTRON, OUTPUT);






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
      uint8_t buffer[256];
      uint8_t dataSrc = CANreceiveBuffer[i][3];
      uint8_t dataPGN = CANreceiveBuffer[i][4];
      uint8_t dataLen = CANreceiveBuffer[i][5];
      buffer[0] = 0x80;
      buffer[1] = 0x81;
      buffer[2] = dataSrc;
      buffer[3] = dataPGN;
      buffer[4] = dataLen;

      if (dataLen > 0) {
        memcpy(&buffer[5], &CANreceiveBuffer[i][6], dataLen);
      }

      uint8_t crc = calculateCRC(buffer, 5 + dataLen);
      buffer[5 + dataLen] = crc;

      if (dataSrc == 123 || (dataSrc == 127 && dataPGN == 239)) {
        SerialPop.write(buffer, 6 + dataLen);
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