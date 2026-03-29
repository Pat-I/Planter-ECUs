void RetrieveRowData() {
  //todo calculate speed (and speed differential lft right)


  for (uint8_t i = 0; i < numPlanterRows; i++) {
    if (sensorNewData[i]) {
      sensorNewData[i] = 0;  // ready for next time
      noInterrupts();
      sensorSeedTimeStable[i] = sensorSeedTime[i];
      interrupts();

#ifdef SERIAL_POP_COUNTER
      SeedCountTotal[i]++;
      Serial.print("seed on row ");
      Serial.println((i + 1));
#endif

      sensorSeedDuration = sensorSeedTimeStable[i] - lastSensorTime[i];
      lastSensorTime[i] = sensorSeedTimeStable[i];

      if (sensorSeedDuration < 999 && sensorSeedDuration > 0)  // valid data
      {
        //check the array
        uint8_t putArrayIndex = sensorAllGapsIndex[i] % 100;

        if (!isRowSeeding[i]) {
          if (SeedPreviousDuration[i] < 250) {
            // two seeds in a row
            isRowSeeding[i] = true;
          }
        }

        if (isRowRecoring[i] && ReceivedFirstSeed[i]) {
          //we do not count the first gap after the seeder has been lowered
          rc_seedCount[i]++;
          // the spacing will overflow over 25.5km/h because of AOGSpeedX10
          actualPlantSpacing = (sensorSeedDuration * (uint32_t)AOGSpeedX10) / 36;

          //put the data in the array
          sensorAllGaps[i][putArrayIndex] = (uint16_t)actualPlantSpacing;
          sensorAllSk[putArrayIndex] = 0;
          sensorAllDbl[putArrayIndex] = 0;
          sensorAllGapsIndex[i]++;
          if (sensorAllGapsIndex[i] > 199) sensorAllGapsIndex[i] = 100;


          if (actualPlantSpacing > seedGapSkip) {
            sk_skips[i]++;
            rc_skips[i]++;
          }
          if (actualPlantSpacing < seedGapDouble) {
            dbl_doubles[i]++;
            rc_doubles[i]++;
          }
        } else {
          ReceivedFirstSeed[i] = true;
        }
      }  //end of "valid data"
      SeedPreviousDuration[i] = sensorSeedDuration;
    }
  }
}