void Summary() {
  if (isTalkingToAiO) SerialPop.print("ESP32-hello");  // for testing on AIO ESP32 slot
  digitalWrite(13, HIGH);

  for (uint8_t i = 0; i < numPlanterRows; i++) {

    uint8_t count = min(sensorAllGapsIndex[i], 99);
    SeedCount[i] = count;
    Skips[i] = 0;
    Doubles[i] = 0;
    avgSpacing[i] = 0;
    singulation[i] = 100;
    population[i] = 0;

    if (count > 0) {
      for (uint8_t j = 0; j < count; j++) {
        Skips[i] += sensorAllSk[j];
        Doubles[i] += sensorAllDbl[j];
        avgSpacing[i] += sensorAllGaps[i][j];
      }

      avgSpacing[i] /= count;
      float sing = 100.0f * (count - (Skips[i] + Doubles[i])) / count;
      singulation[i] = (uint8_t)sing;
      if (avgSpacing[i] > 0) population[i] = 10000000000 / (avgSpacing[i] * rowWidth);
      avgSpacing[i] = min((uint16_t)avgSpacing[i], 255);
    }
  }

#ifdef SEND_POP_PGN
  //build pop PGNs
  for (int i = 0; i < numPlanterRows; i++) {
    popDVD100[i] = population[i] / 100;  // to fit into 2 buffers
  }

  byteIndex = 5;
  for (int i = 0; i < 8; i++) {
    pop_data[byteIndex] += popDVD100[i];
    byteIndex++;
  }  // numPlanterRows

  byteIndex = 5;
  for (int i = 8; i < 16; i++) {
    pop2_data[byteIndex] += popDVD100[i];
    byteIndex++;
  }  // numPlanterRows

  CK_A = 0;

  for (int16_t i = 2; i < pop_dataSize - 1; i++) {
    CK_A = (CK_A + pop_data[i]);
  }
  pop_data[pop_dataSize - 1] = CK_A;
  Serial1.write(pop_data, pop_dataSize);

  CK_A = 0;

  for (int16_t i = 2; i < pop2_dataSize - 1; i++) {
    CK_A = (CK_A + pop2_data[i]);
  }

  pop2_data[pop2_dataSize - 1] = CK_A;
  Serial1.write(pop2_data, pop2_dataSize);

  for (int k = 5; k < 13; k++) {
    pop_data[k] = datazero;
    pop2_data[k] = datazero;
  }
#endif

  //build the spacing PGNs
  byteIndex = 5;
  for (int i = 0; i < 8; i++) {
    space_data[byteIndex] += avgSpacing[i];
    byteIndex++;
  }  // numPlanterRows

  byteIndex = 5;
  for (int i = 8; i < 16; i++) {
    space2_data[byteIndex] += avgSpacing[i];
    byteIndex++;
  }  // numPlanterRows

  CK_A = 0;

  for (int16_t i = 2; i < space_dataSize - 1; i++) {
    CK_A = (CK_A + space_data[i]);
  }
  space_data[space_dataSize - 1] = CK_A;
  SerialPop.write(space_data, space_dataSize);

  CK_A = 0;

  for (int16_t i = 2; i < space2_dataSize - 1; i++) {
    CK_A = (CK_A + space2_data[i]);
  }

  space2_data[space2_dataSize - 1] = CK_A;
  SerialPop.write(space2_data, space2_dataSize);

  for (int k = 5; k < 13; k++) {
    space_data[k] = datazero;
    space2_data[k] = datazero;
  }
  //build singulation PGNs
  byteIndex = 5;
  for (int i = 0; i < 8; i++) {
    sin_data[byteIndex] += singulation[i];
    byteIndex++;
  }  // numPlanterRows

  byteIndex = 5;
  for (int i = 8; i < 16; i++) {
    sin2_data[byteIndex] += singulation[i];
    byteIndex++;
  }  // numPlanterRows

  CK_A = 0;
  for (int16_t i = 2; i < sin_dataSize - 1; i++) {
    CK_A = (CK_A + sin_data[i]);
  }
  sin_data[sin_dataSize - 1] = CK_A;
  SerialPop.write(sin_data, sin_dataSize);

  CK_A = 0;
  for (int16_t i = 2; i < sin2_dataSize - 1; i++) {
    CK_A = (CK_A + sin2_data[i]);
  }
  sin2_data[sin2_dataSize - 1] = CK_A;
  SerialPop.write(sin2_data, sin2_dataSize);

  for (int k = 5; k < 13; k++) {
    sin_data[k] = datazero;
    sin2_data[k] = datazero;
  }


  //build Summary PGN
  uint8_t activeRows = 0;
  uint32_t totalSeedCnt = 0;
  sum_doublesPercentX10 = 0;
  sum_skipPercentX10 = 0;
  sum_singulationX10 = 1000;
  sum_populationD10 = 0;

  for (uint8_t i = 0; i < numPlanterRows; i++) {
    if (SeedCount[i] > 0) {
      activeRows++;
      sum_populationD10 += population[i];
      sum_doublesPercentX10 += Doubles[i];
      sum_skipPercentX10 += Skips[i];
      totalSeedCnt += SeedCount[i];
    }
  }

  if (activeRows > 0) {
    sum_doublesPercentX10 *= 1000;
    sum_doublesPercentX10 /= totalSeedCnt;
    sum_skipPercentX10 *= 1000;
    sum_skipPercentX10 /= totalSeedCnt;
    sum_singulationX10 = 1000 - sum_doublesPercentX10 - sum_skipPercentX10;
    sum_populationD10 /= activeRows;
    sum_populationD10 /= 10;
  }

  uint16_t out_pop = 65535;
  if (sum_populationD10 < 65535) out_pop = sum_populationD10;

  rc_summary[5] = (uint8_t)out_pop;
  rc_summary[6] = out_pop >> 8;

  int16_t out_skips = (int)sum_skipPercentX10;
  rc_summary[7] = (uint8_t)out_skips;
  rc_summary[8] = out_skips >> 8;

  int16_t out_doubles = (int)sum_doublesPercentX10;
  rc_summary[9] = (uint8_t)out_doubles;
  rc_summary[10] = out_doubles >> 8;

  int16_t out_sing = (int)sum_singulationX10;
  rc_summary[11] = (uint8_t)out_sing;
  rc_summary[12] = out_sing >> 8;
  CK_A = 0;

  for (int16_t i = 2; i < rc_summarySize - 1; i++) {
    CK_A = (CK_A + rc_summary[i]);
  }

  rc_summary[rc_summarySize - 1] = CK_A;

  SerialPop.write(rc_summary, rc_summarySize);
}