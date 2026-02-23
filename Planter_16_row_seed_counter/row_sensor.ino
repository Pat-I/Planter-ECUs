void RetrieveRowData() {
  //todo calculate speed (and speed differential lft right)


  for (uint8_t i = 0; i < numPlanterRows; i++) {
    if (sensorNewData[i]) {
      sensorNewData[i] = 0;  // ready for next time
      noInterrupts()
        sensorSeedTimeStable[i] = sensorSeedTime[i];
      interrupts()

        sensorSeedDuration = sensorSeedTimeStable[i] - lastSensorTime[i];
      lastSensorTime[i] = sensorSeedTimeStable[i];

      if (sensorSeedDuration < 999 && sensorSeedDuration > 0)  // valid data
      {
        putArrayIndex = 0;
        //check the array
        if (sensorAllGapsIndex[i] > 99) putArrayIndex = sensorAllGapsIndex[i] - 100;


        if (!isRowSeeding[i]) {
          if (SeedPreviousDuration[i] < 250) {
            // two seeds in a row
            isRowSeeding[i] = true;
          }
        }

        if (isRowRecoring[i] && ReceivedFirstSeed[i]) {
          //we do not count the first gap after the seeder has been lowered

          actualPlantSpacing = sensorSeedDuration * AOGSpeedX10;
          actualPlantSpacing /= 36;

          //put the data in the array
          sensorAllGaps[i][putArrayIndex] = actualPlantSpacing;  //change to dist (seedDistancePut)
          sensorAllSk[putArrayIndex] = 0;
          sensorAllDbl[putArrayIndex] = 0;
          sensorAllGapsIndex[i]++;
          if (sensorAllGapsIndex[i] > 199) sensorAllGapsIndex[i] = 100;


          if (actualPlantSpacing > (uint16_t)seedGapSkip) {
            sk_skips[i]++;
          }
          if (actualPlantSpacing < (uint16_t)seedGapDouble) {
            dbl_doubles[i]++;
          }
        }else{
          ReceivedFirstSeed[i] =true;
        }
      }//end of "valid data"
      SeedPreviousDuration[i] = sensorSeedDuration;

    }
  }
}