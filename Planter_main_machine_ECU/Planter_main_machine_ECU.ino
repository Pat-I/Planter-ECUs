

char arduinoDate[] = "2026-03-28";
char firmwareName[] = "JD1770NT main machine ECU";
char arduinoVersion[] = "v 1.0.4";

/*  PWM Frequency -> 
   *   490hz (default) = 0
   *   122hz = 1
   *   3921hz = 2
   */
#define PWM_Frequency 1

//loop time variables in milliseconds
const byte LOOP_TIME = 100;  // 10Hz
unsigned long lastTime = LOOP_TIME;
unsigned long currentTime = LOOP_TIME;

//communication
uint8_t CANreceiveBuffer[8][255];
uint8_t AOGtoCAN[255] = { 0 };  // Forces all elements to 0
uint8_t AOGtoCANseq = 0;

///////main for the pop serial reading/////////////////////////////////////////////////////////
#define SerialPop Serial1
uint8_t popRxBuffer[2048];
uint8_t popTxBuffer[2048];
uint32_t bautPop = 460800;
//Parsing PGN
bool isHeaderFound = false;
int16_t tempHeader = 0;
bool isLengthFound = false;
int header = 0;
int temp = 0;
uint8_t serialSource = 0;
uint8_t serialPgn = 0;
uint8_t serialLength = 0;
uint8_t serialData[256];  //just to be sure it's long enough
uint8_t serialCRC = 0;
////////////////////////////////////////////////////////////////////////////////////////////


//define inputs and outputs
#define PWM1_CYTRON 3
#define DIR1_CYTRON 4

#define BOUTON_UP 6
#define BOUTON_DOWN 7

#define POTO_UP A0
#define POTO_DOWN A1

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
  pinMode(BOUTON_UP, INPUT);  //INSTEAD INPUT_PULLUP, not needed?
  pinMode(BOUTON_DOWN, INPUT);
  pinMode(DIR1_CYTRON, OUTPUT);






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
    analogRead(BOUTON_UP);
    analogWrite(PWM1_CYTRON, 128);







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

  if (SerialPop.available() > 2 && isHeaderFound && !isLengthFound) {
    serialSource = SerialPop.read();
    serialPgn = SerialPop.read();
    serialLength = SerialPop.read();
    isLengthFound = true;
  }

  if (SerialPop.available() > serialLength && isHeaderFound && isLengthFound) {
    //We have all data, reset for next time
    isHeaderFound = false;
    isLengthFound = false;

    for (uint8_t i = 0; i < serialLength; i++) {
      serialData[i] = SerialPop.read();
    }
    serialCRC = SerialPop.read();

    //todo: check CRC, if bad, return, if good continue

    //send to CAN3
    AOGtoCAN[0] = 0x80;
    AOGtoCAN[1] = 0x81;
    AOGtoCAN[2] = serialSource;
    AOGtoCAN[3] = serialPgn;
    AOGtoCAN[4] = serialLength;
    for (uint8_t i = 0; i < serialLength; i++) {
      AOGtoCAN[i + 5] = serialData[i];
    }
    AOGtoCAN[serialLength + 5] = serialCRC;

    EncodeAOGtoCAN();
  }
  /////////end of serial to CAN//////////////////////
}  // end of loop

void CheckDataFromCAN() {
  for (uint8_t i = 0; i < 8; i++) {
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
        //memcpy(&buffer[5], data, dataLen);
        for (uint8_t j = 0; j < dataLen; j++) {
          buffer[j + 5] = CANreceiveBuffer[i][j + 6];
        }
      }

      uint8_t crc = calculateCRC(buffer, 5 + dataLen);
      buffer[5 + dataLen] = crc;

      if (dataSrc == 123 || (dataSrc == 127 && dataPGN == 239)) SerialPop.write(buffer, 6 + dataLen);
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