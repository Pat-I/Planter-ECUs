

char arduinoDate[] = "2026-03-27";
char arduinoVersion[] = "v 1.0.4";

//#define SERIAL_POP_COUNTER  //show the number of seed passed per row in the serial monitor,

//#define SEND_POP_PGN  // send PGN
/*
This code use work from Jim from outfarming.com
It send:
PGN E1 and E2 (225 226) pop per row X1000
PGN CA and CB (202 203) spacing per row -> spacing in mm 0 to 255
PGN CC and CD (204 205) singulation per row -> spacing in mm 0 to 100%
PGN E3 and E4 (227 228) Doubles and Skips, 0 to 7 per row in the array time laps
PGN E5 229 planter summary of the last 100 seeds from all active rows , popHaD10, SkipsPercentX10, DoublesPercentX10, SingulationPercentX10

also it should (to add) respond by E0 (224) when E0 is received
it receive also E9 (233)

It need:
PGN EF (239) from AGO for speed and section status
TO add:  real PGN 7FE5 (127,229)  (64 sections PGN) sections end speeds to do row side speed compensation

*/



#define SerialPop Serial1
uint32_t bautPop = 460800;
uint8_t popRxBuffer[2048];
uint8_t popTxBuffer[2048];
bool isTalkingToAiO = true;  //set to true only if talking to the AiO ESP32 slot
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

#ifdef SERIAL_POP_COUNTER
//teensy usage calc
uint32_t cycles_max = 0;    // Référence (CPU à 0%)
uint32_t compteur = 0;      // Compteur actuel
uint32_t timer_mesure = 0;  // Chronomètre
#endif
//Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency);
extern float tempmonGetTemp(void);

const uint8_t datazero = 0;
uint8_t CK_A = 0;

uint8_t sin_data[] = { 0x80, 0x81, 0x7b, 0xCD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t sin_dataSize = sizeof(sin_data);

uint8_t sin2_data[] = { 0x80, 0x81, 0x7b, 0xCC, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t sin2_dataSize = sizeof(sin2_data);

uint8_t space_data[] = { 0x80, 0x81, 0x7b, 0xCB, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t space_dataSize = sizeof(space_data);

uint8_t space2_data[] = { 0x80, 0x81, 0x7b, 0xCA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t space2_dataSize = sizeof(space2_data);

uint8_t rc_data[] = { 0x80, 0x81, 0x7b, 0xE6, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t rc_dataSize = sizeof(rc_data);

uint8_t rc_summary[] = { 0x80, 0x81, 0x7b, 0xE5, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t rc_summarySize = sizeof(rc_summary);

uint8_t sk_data[] = { 0x80, 0x81, 0x7b, 0xE4, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t sk_dataSize = sizeof(sk_data);

uint8_t dbl_data[] = { 0x80, 0x81, 0x7b, 0xE3, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t dbl_dataSize = sizeof(dbl_data);

#ifdef SEND_POP_PGN
uint8_t pop_data[] = { 0x80, 0x81, 0x7b, 0xE2, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t pop_dataSize = sizeof(pop_data);

uint8_t pop2_data[] = { 0x80, 0x81, 0x7b, 0xE1, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t pop2_dataSize = sizeof(pop2_data);
#endif

uint8_t feedback[] = { 0x80, 0x81, 0x7b, 0xE0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t feedbackSize = sizeof(feedback);

uint8_t rowStatus[] = { 0x80, 0x81, 0x7b, 0xF0, 4, 1, 0, 0, 0, 15 };// for 16 row only
int16_t rowStatusSize = sizeof(rowStatus);

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
uint32_t seedDebounceTime = 2;                                                            //Debounce time in ms after seed detection, 3ms is about the time a seed passes by?
volatile bool sensorNewData[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };       //A seed has been read
uint32_t lastSensorTime[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };           //previous time
volatile uint32_t sensorSeedTime[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //Actual time of seed detection

uint32_t sensorSeedTimeStable[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //Actual time of seed detection but not volatile

//storage variables
uint16_t rowWidth = 762;  // in mm
int numPlanterRows = 16;
uint16_t doublesFactor = 29;
uint32_t targetPopulation = 84000;  //per Ha
//float targetSpeed = 4.9f;
//float AOGSpeed = 0.0f;
uint16_t AOGSpeedX10 = 0;
int isMetric = 1;
uint32_t seedGap = 0;
uint32_t seedGapSkip = 0;
uint32_t seedGapDouble = 0;
//uint16_t doublePlantSpacing = 0;
uint32_t actualPlantSpacing = 0;

uint32_t sensorSeedDuration;                                                           //time betwen 2 seeds
uint32_t SeedPreviousDuration[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //time betwen 2 seeds
uint16_t sensorAllGaps[16][100];
uint8_t sensorAllGapsIndex[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t sensorAllSk[100];
uint8_t sensorAllDbl[100];
bool isRowSeeding[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };   //this is when at least 2 seeds dropped the last 500ms, to send back the section status
bool isRowRecoring[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //this is went AOG command on and the planter is lowered.
bool ReceivedFirstSeed[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
bool isPlanterLowered = true;
uint8_t sectionStatus[] = { 0, 0 };

uint8_t millisSectionStatus = 0;
//Summary
#ifdef SERIAL_POP_COUNTER
uint16_t SeedCountTotal[16];
#endif
uint8_t millisAtSCount = 0;
uint8_t SeedCount[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t Skips[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t Doubles[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint32_t avgSpacing[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t singulation[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
#ifdef SEND_POP_PGN
uint16_t popDVD100[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
#endif
uint32_t population[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint32_t sum_skipPercentX10 = 0;
uint32_t sum_doublesPercentX10 = 0;
uint32_t sum_singulationX10 = 0;
uint32_t sum_populationD10 = 0;
uint8_t byteIndex = 5;
//skip double detail data
uint8_t millisForArray = 5;
int sk_skips[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int dbl_doubles[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t millisAtDblCount = 0;
int sk_byteIndex = 12;
int dbl_byteIndex = 12;

void setup() {
  // Core at 150 MHz (To reduce heat)
  set_arm_clock(150000000);
  // put your setup code here, to run once:
  delay(100);
  Serial.print("CPU speed set to: ");
  Serial.println(F_CPU_ACTUAL);

#ifdef SERIAL_POP_COUNTER
  // 1. Calibration : Compter les boucles vides pendant 500ms
  uint32_t start = millis();
  while (millis() - start < 1000) {
    compteur++;
  }
  cycles_max = compteur;  // On a notre 100% de repos
  compteur = 0;
  timer_mesure = millis();
#endif

  Serial.begin(115200);
  SerialPop.begin(bautPop);
  SerialPop.addMemoryForRead(popRxBuffer, sizeof(popRxBuffer));
  SerialPop.addMemoryForWrite(popTxBuffer, sizeof(popTxBuffer));

  //delay(1000);
  delay(100);
  pinMode(PinIN[0], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[0]), ISR0, RISING);
  pinMode(PinIN[1], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[1]), ISR1, RISING);
  pinMode(PinIN[2], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[2]), ISR2, RISING);
  pinMode(PinIN[3], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[3]), ISR3, RISING);
  pinMode(PinIN[4], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[4]), ISR4, RISING);
  pinMode(PinIN[5], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[5]), ISR5, RISING);
  pinMode(PinIN[6], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[6]), ISR6, RISING);
  pinMode(PinIN[7], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[7]), ISR7, RISING);
  pinMode(PinIN[8], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[8]), ISR8, RISING);
  pinMode(PinIN[9], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[9]), ISR9, RISING);
  pinMode(PinIN[10], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[10]), ISR10, RISING);
  pinMode(PinIN[11], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[11]), ISR11, RISING);
  pinMode(PinIN[12], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[12]), ISR12, RISING);
  pinMode(PinIN[13], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[13]), ISR13, RISING);
  pinMode(PinIN[14], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[14]), ISR14, RISING);
  pinMode(PinIN[15], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinIN[15]), ISR15, RISING);

  pinMode(13, OUTPUT);
  Serial.println("Planter Seed Counter");
  Serial.println(arduinoVersion);
  Serial.println(arduinoDate);
#ifdef SERIAL_POP_COUNTER
  Serial.print("Score de repos : ");
  Serial.println(cycles_max);
#endif
}

void loop() {
  // put your main code here, to run repeatedly:

  //Loop triggers every 100 msec
  currentTime = millis();

#ifdef SERIAL_POP_COUNTER
  compteur++;  // Load calculator

  if (millis() - timer_mesure >= 1000) {
    // Calcul de l'inactivité : (cycles actuels / cycles max) * 100
    float idle = (float)compteur / (float)cycles_max;
    idle *= 100;
    float charge = 100.0 - idle;
    timer_mesure = millis();

    if (charge < 0) charge = 0;  // Ajustement si légère fluctuation

    Serial.print("row1= ");
    Serial.print(SeedCountTotal[0]);
    Serial.print(", row2= ");
    Serial.print(SeedCountTotal[1]);
    Serial.print(", row3= ");
    Serial.print(SeedCountTotal[2]);
    Serial.print(", row4= ");
    Serial.print(SeedCountTotal[3]);
    Serial.print(", row5= ");
    Serial.print(SeedCountTotal[4]);
    Serial.print(", row6= ");
    Serial.print(SeedCountTotal[5]);
    Serial.print(", row7= ");
    Serial.print(SeedCountTotal[6]);
    Serial.print(", row8= ");
    Serial.print(SeedCountTotal[7]);
    Serial.print(", row9= ");
    Serial.print(SeedCountTotal[8]);
    Serial.print(", row10= ");
    Serial.print(SeedCountTotal[9]);
    Serial.print(", row11= ");
    Serial.print(SeedCountTotal[10]);
    Serial.print(", row12= ");
    Serial.print(SeedCountTotal[11]);
    Serial.print(", row13= ");
    Serial.print(SeedCountTotal[12]);
    Serial.print(", row14= ");
    Serial.print(SeedCountTotal[13]);
    Serial.print(", row15= ");
    Serial.print(SeedCountTotal[14]);
    Serial.print(", row16= ");
    Serial.print(SeedCountTotal[15]);

    Serial.print(", CPU : ");

    Serial.print(charge);
    Serial.print(" %");

    compteur = 0;

    float celsius = tempmonGetTemp();
    Serial.print(", T: ");
    Serial.print(celsius);
    Serial.println(" °C");
  }
#endif

  if (currentTime - lastTime >= LOOP_TIME) {
    lastTime = currentTime;

    millisSectionStatus++;
    millisAtSCount++;
    millisAtDblCount++;

    //check if the rows are still seeding
    for (uint8_t i = 0; i < numPlanterRows; i++) {
      if (isRowSeeding[i]) {
        if (currentTime - sensorSeedTimeStable[i] > 200) {
          // over half a second witout seed so turn section off
          isRowSeeding[i] = false;
          //just reset the index
          //sensorAllTimesIndex[i] = 0;// not here, here we send the section off to AOG
        }
      }
    }
    //send status to AOG
    SendRowStatus();
    // check if a field is connected
    if (millisSectionStatus > 3) {
      millisSectionStatus = 0;  // wait 0.4 sec to do it again if still no connection
      for (uint8_t i = 0; i < numPlanterRows; i++) {
        isRowRecoring[i] = false;
        ReceivedFirstSeed[i] = false;
        sensorAllGapsIndex[i] = 0;  //reset the array
        sensorAllGaps[i][0] = 0;
        sectionStatus[0] = 0;
        sectionStatus[1] = 0;
      }
    }

    if (millisAtDblCount >= millisForArray) {
      doubledetail();
      skipdetail();
      millisAtDblCount = 0;
    }

    if (millisAtSCount >= 10) {
      Summary();
      millisAtSCount = 0;
    }
  }  // end of 100 ms loop

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

    if (serialSource == 123)  // from AOG Planter monitor
    {
      if (serialPgn == 224)  //E0 PlanterConfigData
      {
        Serial.println("config received");
        digitalToggle(13);
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
        numPlanterRows = (planterSettings.rxNumPlanterRows);
        //targetSpeed = (float)(planterSettings.rxTargetSpeedX10) * 0.1f;
        rowWidth = (planterSettings.rxRowWidthX10Lo | planterSettings.rxRowWidthX10Hi << 8);
        targetPopulation = (planterSettings.rxTargetPopulationLo | planterSettings.rxTargetPopulationHi << 8) * 10.0;
        doublesFactor = planterSettings.rxDoublesFactor;
        isMetric = planterSettings.rxIsMetric;

        //seedGap = 3600000000.0f / ((float)targetPopulation * rowWidth * targetSpeed); //gap in ms = pop/HA x row cm x speed km/h
        seedGap = 10000000000 / (targetPopulation * rowWidth);  //gap in mm = pop/HA x row cm


        seedGapSkip = seedGap * 18;
        seedGapSkip /= 10;
        seedGapDouble = seedGap * doublesFactor;
        seedGapDouble /= 100;
        //doublePlantSpacing = seedGap * 2;
      }

      if (serialPgn == 233)  //E9 PlanterConfigData
      {
        //byte 5
        planterSettings.rxArraySpeed = serialData[0];
        // conversions
        millisForArray = planterSettings.rxArraySpeed;
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
        //Serial.print("Speed= ");
        //Serial.println(AOGSpeedX10);

        //AOGSpeedX10 = (planterSettings.rxSpeedHi | planterSettings.rxSpeedLo << 8);  //
        //AOGSpeed = (float)AOGSpeedX10 * 0.1f;  //

        sectionStatus[0] = serialData[6];
        sectionStatus[1] = serialData[7];
        millisSectionStatus = 0;
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
      uint8_t byteNbr = i / 8;

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

void SendRowStatus() {
  //uint8_t rowStatus[] = { 0x80, 0x81, 0x7b, 0xF0, 4, stat, sect, 1to8, 9to16, 15 };
  //int16_t rowStatusSize = sizeof(rowStatus);
  rowStatus[6] = numPlanterRows;
  for (uint8_t i = 0; i < numPlanterRows; i++) {
    rowStatus[7 + i / 8] |= isRowSeeding[i] << (i - (i / 8) * 8);  //i - (i / 8) * 8
  }
  CK_A = 0;

  for (int16_t i = 2; i < rowStatusSize - 1; i++) {
    CK_A = (CK_A + rowStatus[i]);
  }

  rowStatus[rowStatusSize - 1] = CK_A;

  SerialPop.write(rowStatus, rowStatusSize);
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
  if ((millis() - sensorSeedTime[2]) > seedDebounceTime) {
    sensorSeedTime[2] = millis();
    sensorNewData[2] = 1;
  }
}
void ISR3() {
  if ((millis() - sensorSeedTime[3]) > seedDebounceTime) {
    sensorSeedTime[3] = millis();
    sensorNewData[3] = 1;
  }
}
void ISR4() {
  if ((millis() - sensorSeedTime[4]) > seedDebounceTime) {
    sensorSeedTime[4] = millis();
    sensorNewData[4] = 1;
  }
}
void ISR5() {
  if ((millis() - sensorSeedTime[5]) > seedDebounceTime) {
    sensorSeedTime[5] = millis();
    sensorNewData[5] = 1;
  }
}
void ISR6() {
  if ((millis() - sensorSeedTime[6]) > seedDebounceTime) {
    sensorSeedTime[6] = millis();
    sensorNewData[6] = 1;
  }
}
void ISR7() {
  if ((millis() - sensorSeedTime[7]) > seedDebounceTime) {
    sensorSeedTime[7] = millis();
    sensorNewData[7] = 1;
  }
}
void ISR8() {
  if ((millis() - sensorSeedTime[8]) > seedDebounceTime) {
    sensorSeedTime[8] = millis();
    sensorNewData[8] = 1;
  }
}
void ISR9() {
  if ((millis() - sensorSeedTime[9]) > seedDebounceTime) {
    sensorSeedTime[9] = millis();
    sensorNewData[9] = 1;
  }
}
void ISR10() {
  if ((millis() - sensorSeedTime[10]) > seedDebounceTime) {
    sensorSeedTime[10] = millis();
    sensorNewData[10] = 1;
  }
}
void ISR11() {
  if ((millis() - sensorSeedTime[11]) > seedDebounceTime) {
    sensorSeedTime[11] = millis();
    sensorNewData[11] = 1;
  }
}
void ISR12() {
  if ((millis() - sensorSeedTime[12]) > seedDebounceTime) {
    sensorSeedTime[12] = millis();
    sensorNewData[12] = 1;
  }
}
void ISR13() {
  if ((millis() - sensorSeedTime[13]) > seedDebounceTime) {
    sensorSeedTime[13] = millis();
    sensorNewData[13] = 1;
  }
}
void ISR14() {
  if ((millis() - sensorSeedTime[14]) > seedDebounceTime) {
    sensorSeedTime[14] = millis();
    sensorNewData[14] = 1;
  }
}
void ISR15() {
  if ((millis() - sensorSeedTime[15]) > seedDebounceTime) {
    sensorSeedTime[15] = millis();
    sensorNewData[15] = 1;
  }
}