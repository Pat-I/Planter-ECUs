void sendDetail(uint8_t* sourceArray, uint8_t* dataBuffer) {

  for (int i = 0; i < numPlanterRows; i += 2) {
    uint8_t val = (sourceArray[i] > 7) ? 7 : sourceArray[i];
    val <<= 4;

    if (i + 1 < numPlanterRows) {
      val |= (sourceArray[i+1] > 7) ? 7 : sourceArray[i+1];
    }
    dataBuffer[5 + (i >> 1)] = val;
  }

  memset(sourceArray, 0, numPlanterRows);

  uint8_t ck_a = 0;
  for (int i = 2; i < 13; i++) {
    ck_a += dataBuffer[i];
  }
  dataBuffer[13] = ck_a;

  // 4. Envoi via Serial
  SerialPop.write(dataBuffer, 14);

  memset(&dataBuffer[5], 0, 8); 
}