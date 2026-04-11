

char arduinoDate[] = "2026-04-10";
char firmwareName[] = "JD1770NT rowCommand ECU";
char arduinoVersion[] = "v 1.0.0";
/*
Teensy Pinout
GND
0 (ECU pin 25-53 IN) not used
1 (ECU pin 26-54 OUT) not used
2 (ECU pin 36)Pin9 output
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
23 A9 (ECU pin 14) Pin16 output
22 A8 (ECU pin 42) Pin15 output
21
20
19 A5 (ECU pin 41) Pin14 output
18 A4 (ECU pin 40) Pin13 output
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
37 (ECU pin 1) Pin12 output
36 (ECU pin 1) Pin11 output
35 (ECU pin 52 A) TX8 not used
34 (ECU pin 24 B) RX8 not used
33 (ECU pin 1) Pin10 output
*/
uint8_t clutch[] = { 3, 4, 5, 6, 7, 8, 9, 10, 2, 33, 36, 37, 18, 19, 22, 23 };

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

//communication
uint8_t CANreceiveBuffer[16][288];
uint8_t AOGtoCAN[288] = { 0 };  // Forces all elements to 0
uint8_t AOGtoCANseq = 0;
void EncodeAOGtoCAN(const uint8_t* data, uint8_t dataLen, bool isSentToAOG = true);  //to make the compiler happy, probably because of the optional argument

//define inputs and outputs
//outputs
//#define PWM1_CYTRON 3
//#define DIR1_CYTRON 4

//digital inputs
//#define BOUTON_UP 6
//#define BOUTON_DOWN 7

//analog inputs
//#define POTO_UP A0
//#define POTO_DOWN A1

//input/output variables
bool isPlanterLowered = true;
uint8_t numPlanterRows = 16;
uint8_t heightPlanter = 0;
uint8_t onThreshold = 50;
uint8_t offThreshold = 100;
uint8_t AOGSpeedX10 = 0;
uint8_t millisSectionStatus = 0;
bool isClutchPowered[16] = { 0 };
uint8_t sectionStatus[2] = { 0 };
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
  for (uint8_t i = 0; i < numPlanterRows; i++) {
    pinMode(clutch[i], OUTPUT);
  };

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
    // check if a field is connected
    if (millisSectionStatus > 3) {
      millisSectionStatus = 0;  // wait 0.4 sec to do it again if still no connection

      memset(isClutchPowered, 0, numPlanterRows);
      SetClutches();
    }

    CanCheckOldArray();
    //analogRead(BOUTON_UP);
    //analogWrite(PWM1_CYTRON, 128);

  }  // end of 100 ms loop


  CanDecode();
  //to add: read the CANreceiveBuffer
  CheckDataFromCAN();
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

      //read the revelent PGNs

      if (dataSrc == 123)  // 7B from AOG Planter monitor
      {
        if (dataPGN == 160)  //A0 Height
        {
          //dont read 0 and 1 raw height
          heightPlanter = buffer[7];
          //no 3
          onThreshold = buffer[9];
          offThreshold = buffer[10];
          //no 6, 7

          if (heightPlanter < offThreshold) isPlanterLowered = true;  //enable the clutch from height above
          if (heightPlanter > offThreshold) isPlanterLowered = false;
        }
      }

      if (dataSrc == 127)  //Data from AOG
      {
        if (dataPGN == 239)  //FE autoSteerData
        {
          //Byte 5
          //planterSettings.rxSpeedHi = serialData[0];

          //Byte 6
          //planterSettings.rxSpeedLo = serialData[1];
          AOGSpeedX10 = buffer[6];
          //Serial.print("Speed= ");
          //Serial.println(AOGSpeedX10);

          //AOGSpeedX10 = (planterSettings.rxSpeedHi | planterSettings.rxSpeedLo << 8);  //
          //AOGSpeed = (float)AOGSpeedX10 * 0.1f;  //

          sectionStatus[0] = buffer[11];
          sectionStatus[1] = buffer[12];
          millisSectionStatus = 0;
          CheckRowStatus();
        }
      }
    }
  }
}

void CheckRowStatus() {
  if (isPlanterLowered) {
    //check if section is on
    for (uint8_t i = 0; i < numPlanterRows; i++) {

      isClutchPowered[i] = !bitRead(sectionStatus[i / 8], i % 8);
    }
  } else {
    //raised, stop recording
    memset(isClutchPowered, 0, numPlanterRows);
    //memset(ReceivedFirstSeed, 0, numPlanterRows);
    //memset(sensorAllGapsIndex, 0, numPlanterRows);
  }

  SetClutches();
}

void SetClutches() {
  for (uint8_t i = 0; i < numPlanterRows; i++) {
    digitalWrite(clutch[i], isClutchPowered[i]);
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