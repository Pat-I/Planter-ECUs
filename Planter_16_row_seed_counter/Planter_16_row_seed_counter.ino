

char arduinoDate[] = "2025-01-26";
char arduinoVersion[] = "v 1.0.0";

/*
population in seed/ha = (seed count x 472440945)/(time is ms x speed in (km/h)X10)
*/
//const uint32_t populationConstant = 472440945;
//uint32_t PopulationCalc = 0; // PopulationCalc = populationConstant / time, then multi by seedCnt, then / speedX10
//uint16_t RowPop[16];//max is 65 535 , >>4 divide by 16



//loop time variables in milliseconds
const byte LOOP_TIME = 100;  // 10Hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

struct Storage {
  uint8_t rxNumPlanterRows = 16;
  uint8_t rxTargetSpeedX10 = 48;
  uint8_t txSpeed = 0;
  uint8_t rxRowWidthX10Hi = 1;
  uint8_t rxRowWidthX10Lo = 54;
  uint8_t rxTargetPopulationHi = 12;
  uint8_t rxTargetPopulationLo = 128;
  uint8_t rxDoublesFactor = 55;
  uint8_t rxSpeedHi = 0;
  uint8_t rxSpeedLo = 0;
  bool rxIsMetric = false;
  uint8_t rxArraySpeed = 1;
};
Storage planterSettings;  //14 bytes

const uint8_t datazero = 0;

uint8_t sin_data[] = { 0x80, 0x81, 0x7b, 0xE8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t sin_dataSize = sizeof(sin_data);

uint8_t sin2_data[] = { 0x80, 0x81, 0x7b, 0xE7, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t sin2_dataSize = sizeof(sin2_data);

uint8_t rc_data[] = { 0x80, 0x81, 0x7b, 0xE6, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t rc_dataSize = sizeof(rc_data);

uint8_t rc_summary[] = { 0x80, 0x81, 0x7b, 0xE5, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t rc_summarySize = sizeof(rc_summary);

uint8_t sk_data[] = { 0x80, 0x81, 0x7b, 0xE4, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t sk_dataSize = sizeof(sk_data);

uint8_t dbl_data[] = { 0x80, 0x81, 0x7b, 0xE3, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t dbl_dataSize = sizeof(dbl_data);

uint8_t pop_data[] = { 0x80, 0x81, 0x7b, 0xE2, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t pop_dataSize = sizeof(pop_data);

uint8_t pop2_data[] = { 0x80, 0x81, 0x7b, 0xE1, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t pop2_dataSize = sizeof(pop2_data);

uint8_t feedback[] = { 0x80, 0x81, 0x7b, 0xE0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t feedbackSize = sizeof(rc_summary);
//Parsing PGN
bool isHeaderFound = false;
int16_t tempHeader = 0;
bool isLengthFound = false;
int header = 0;
int temp = 0;
uint8_t serialSource = 0;
uint8_t serialPgn = 0;
uint8_t serialLength = 0;
uint8_t serialData[64];  //just to be sure it's long enough
uint8_t serialCRC = 0;

//Pins
//Sensor Pins
uint8_t PinIN[] = { 34, 33, 36, 35, 38, 37, 40, 39, 14, 41, 16, 15, 18, 17, 20, 19 };


//Sensor logic
uint32_t seedDebounceTime = 10;                                                           //Debounce time in ms after seed detection, 10ms for up to 400 000 seed/ha at 10km/h or 250k at 16
volatile bool sensorNewData[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };       //A seed has been read
uint32_t lastSensorTime[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };           //previous time
volatile uint32_t sensorSeedTime[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //Actual time of seed detection

uint32_t sensorSeedTimeStable[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //Actual time of seed detection but not volatile

//storage variables
float rowWidth = 76.2f;
int numPlanterRows = 16;
float doublesFactor = .29f;
float targetPopulation = 84000.0f;
float targetSpeed = 4.9f;
float AOGSpeed = 0.0f;
uint16_t AOGSpeedX10 = 0;
int isMetric = 1;
long seedGap = 0;
long seedGapSkip = 0;
long seedGapDouble = 0;
uint16_t doublePlantSpacing = 0;
uint32_t actualPlantSpacing = 0;

uint16_t sensorSeedDuration;                                                           //time betwen 2 seeds
uint16_t SeedPreviousDuration[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //time betwen 2 seeds
uint16_t sensorAllGaps[16][100];
uint8_t sensorAllGapsIndex[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t putArrayIndex = 0;                                                  //just to use with sensorAllTimesIndex
uint8_t sensorAllSk[100];
uint8_t sensorAllDbl[100];
bool isRowSeeding[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };   //this is when at least 2 seeds dropped the last 500ms, to send back the section status
bool isRowRecoring[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //this is went AOG command on and the planter is lowered.
bool ReceivedFirstSeed[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
bool isPlanterLowered = true;
uint8_t sectionStatus[] = {0,0};

unsigned long millisSectionStatus = 0;
//Summary
unsigned long millisAtSCount = 0;
uint8_t SeedCount[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t Skips[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t Doubles[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t avgSpacing[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t singulation[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t popDVD100[]  = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float population[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint32_t sum_skipPercentX100 = 0;
uint32_t sum_doublesPercentX100 = 0;
uint32_t sum_singulationX100 = 0;
uint32_t sum_populationD10 = 0;
uint8_t byteIndex = 5;
//skip double detail data
unsigned long millisForArray = 5000;
int sk_skips[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int dbl_doubles[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long millisAtSkipCount = 0;
unsigned long millisAtDblCount = 0;
int sk_byteIndex = 12;
int dbl_byteIndex = 12;

void setup() {
  // put your setup code here, to run once:
  delay(100);
  Serial.begin(115200);
  Serial1.begin(115200);

  //delay(1000);
  delay(100);
  pinMode(PinIN[0], INPUT);
  attachInterrupt(digitalPinToInterrupt(PinIN[0]), ISR0, RISING);
  pinMode(PinIN[1], INPUT);
  attachInterrupt(digitalPinToInterrupt(PinIN[1]), ISR1, RISING);
  pinMode(PinIN[2], INPUT);
  attachInterrupt(digitalPinToInterrupt(PinIN[2]), ISR2, RISING);
  /*
    pinMode(PinIN[3], INPUT);
    attachInterrupt(digitalPinToInterrupt(PinIN[3]), ISR3, RISING);
    pinMode(PinIN[4], INPUT);
    attachInterrupt(digitalPinToInterrupt(PinIN[4]), ISR4, RISING);
    pinMode(PinIN[5], INPUT);
    attachInterrupt(digitalPinToInterrupt(PinIN[5]), ISR5, RISING);
    pinMode(PinIN[6], INPUT);
    attachInterrupt(digitalPinToInterrupt(PinIN[6]), ISR6, RISING);
    pinMode(PinIN[7], INPUT);
    attachInterrupt(digitalPinToInterrupt(PinIN[7]), ISR7, RISING);
    pinMode(PinIN[8], INPUT);
    attachInterrupt(digitalPinToInterrupt(PinIN[8]), ISR8, RISING);
    pinMode(PinIN[9], INPUT);
    attachInterrupt(digitalPinToInterrupt(PinIN[9]), ISR9, RISING);
    pinMode(PinIN[10], INPUT);
    attachInterrupt(digitalPinToInterrupt(PinIN[10]), ISR10, RISING);
    pinMode(PinIN[11], INPUT);
    attachInterrupt(digitalPinToInterrupt(PinIN[11]), ISR11, RISING);
    pinMode(PinIN[12], INPUT);
    attachInterrupt(digitalPinToInterrupt(PinIN[12]), ISR12, RISING);
    pinMode(PinIN[13], INPUT);
    attachInterrupt(digitalPinToInterrupt(PinIN[13]), ISR13, RISING);
    pinMode(PinIN[14], INPUT);
    attachInterrupt(digitalPinToInterrupt(PinIN[14]), ISR14, RISING);
    pinMode(PinIN[15], INPUT);
    attachInterrupt(digitalPinToInterrupt(PinIN[15]), ISR15, RISING);
  */
  Serial.println("Planter Seed Counter");
  Serial.println(arduinoVersion);
  Serial.println(arduinoDate);
}

void loop() {
  // put your main code here, to run repeatedly:

  //Loop triggers every 100 msec
  currentTime = millis();

  if (currentTime - lastTime >= LOOP_TIME) {
    lastTime = currentTime;

    //calculate the population, all the values to a 0, 5 minimum, check skip and double


    //check if the rows are still seeding
    for (uint8_t i = 0; i < numPlanterRows; i++) {
      if (isRowSeeding[i]) {
        if (currentTime - sensorSeedTimeStable[i] > 500) {
          // over half a second witout seed so turn section off
          isRowSeeding[i] = false;
          //just reset the index
          //sensorAllTimesIndex[i] = 0;// not here, here we send the section off to AOG
        }
      }
    }
    // check if a field is connected
    if ((millis() - millisSectionStatus) > 250) {
      for (uint8_t i = 0; i < numPlanterRows; i++) {
        isRowRecoring[i] = false;
        ReceivedFirstSeed[i] = false;
        sensorAllGapsIndex[i] = 0;  //reset the array
        sensorAllGaps[i][0] = 0;
        sectionStatus[0] = 0;
        sectionStatus[1] = 0;
      }
    }


    doubledetail();
    skipdetail();
    Summary();
  }  // end of 100 ms loop

  //This runs continuously, not timed //// Serial Receive Data/Settings /////////////////
  // if there's data available, read a packet

  if (Serial1.available() > 0 && !isHeaderFound) {
    temp = Serial1.read();
    header = tempHeader << 8 | temp;            //high,low bytes to make int
    tempHeader = temp;                          //save for next time
    if (header == 32897) isHeaderFound = true;  //Do we have a match?
  }

  if (Serial1.available() > 2 && isHeaderFound && !isLengthFound) {
    serialSource = Serial1.read();
    serialPgn = Serial1.read();
    serialLength = Serial1.read();
    isLengthFound = true;
  }

  if (Serial1.available() > serialLength && isHeaderFound && isLengthFound) {
    //We have all data, reset for next time
    isHeaderFound = false;
    isLengthFound = false;

    for (uint8_t i = 0; i < serialLength; i++) {
      serialData[i] = Serial1.read();
    }
    serialCRC = Serial1.read();

    //todo: check CRC, if bad, return, if good continue

    if (serialSource == 123)  // from AOG Planter monitor
    {
      if (serialPgn == 224)  //E0 PlanterConfigData
      {
        digitalWrite(13, LOW);
        //byte 5
        planterSettings.rxNumPlanterRows = serialData[0];  //16

        // byte 6
        planterSettings.rxTargetSpeedX10 = serialData[1];  //47

        //byte 7
        planterSettings.rxRowWidthX10Hi = serialData[2];  // 1

        //byte 8
        planterSettings.rxRowWidthX10Lo = serialData[3];  // 54

        //Byte 9
        planterSettings.rxTargetPopulationHi = serialData[4];  // 136

        //Byte 10
        planterSettings.rxTargetPopulationLo = serialData[5];  // 184

        //Byte 11
        planterSettings.rxDoublesFactor = serialData[6];  // 184

        //Byte 12
        planterSettings.rxIsMetric = serialData[7];  // 184
        // conversions
        numPlanterRows = (float)(planterSettings.rxNumPlanterRows);
        targetSpeed = (float)(planterSettings.rxTargetSpeedX10) * 0.1f;
        rowWidth = (float)(planterSettings.rxRowWidthX10Lo | planterSettings.rxRowWidthX10Hi << 8) * 0.1f;
        targetPopulation = (float)((planterSettings.rxTargetPopulationLo | planterSettings.rxTargetPopulationHi << 8) * 10.0f);
        doublesFactor = (float)(planterSettings.rxDoublesFactor) / 100.0f;
        isMetric = planterSettings.rxIsMetric;

        //seedGap = 3600000000.0f / ((float)targetPopulation * rowWidth * targetSpeed); //gap in ms = pop/HA x row cm x speed km/h
        seedGap = 1000000000.0f / ((float)targetPopulation * rowWidth);  //gap in mm = pop/HA x row cm


        seedGapSkip = seedGap * 1.8f;
        seedGapDouble = seedGap * planterSettings.rxDoublesFactor / 100.0f;
        doublePlantSpacing = seedGap * 2;
      }

      if (serialPgn == 233)  //E0 PlanterConfigData
      {
        //byte 5
        planterSettings.rxArraySpeed = serialData[0];
        // conversions
        millisForArray = (int)planterSettings.rxArraySpeed * 1000;
      }
    }  // recv data

    if (serialSource == 127)  //Data from AOG
    {
      if (serialPgn == 239)  //FE autoSteerData
      {
        //Byte 5
        //planterSettings.rxSpeedHi = serialData[0];

        //Byte 6
        //planterSettings.rxSpeedLo = serialData[1];
        AOGSpeedX10 = serialData[1];

        //AOGSpeedX10 = (planterSettings.rxSpeedHi | planterSettings.rxSpeedLo << 8);  //
        AOGSpeed = (float)AOGSpeedX10 * 0.1f;  //

        sectionStatus[0] = serialData[6];
        sectionStatus[1] = serialData[7];
        millisSectionStatus = millis();
        CheckRowStatus();
      }
    }
  }

  RetrieveRowData();
}  // end of loop

void CheckRowStatus() {
  if (isPlanterLowered) {
    //check if section is on
    for (uint8_t i = 0; i < numPlanterRows; i++) {
      uint8_t byteNbr = i/8;

      isRowRecoring[i] = bitRead(sectionStatus[byteNbr], i - byteNbr * 8);
      if (!isRowRecoring[i]) ReceivedFirstSeed[i] = false;
    }
  } else {
    //raised, stop recording
    for (uint8_t i = 0; i < numPlanterRows; i++) {
      isRowRecoring[i] = false;
      ReceivedFirstSeed[i] = false;
    }
  }
}

void ISR0() {
  if ((millis() - sensorSeedTime[0]) > seedDebounceTime) {
    sensorSeedTime[0] = millis();
    sensorNewData[0] = 1;
  }
}
void ISR1() {
  if ((millis() - sensorSeedTime[1]) > seedDebounceTime) {
    sensorSeedTime[1] = millis();
    sensorNewData[1] = 1;
  }
}
void ISR2() {
  if ((millis() - sensorSeedTime[1]) > seedDebounceTime) {
    sensorSeedTime[1] = millis();
    sensorNewData[1] = 1;
  }
}