void doubledetail() {

  uint8_t dbl_byteIndex = 5;

  for (int j = 0; j < numPlanterRows; j++) {
    if (dbl_doubles[j] > 0) {
      (dbl_doubles[j] > 7) ? dbl_data[dbl_byteIndex] = 7 : dbl_data[dbl_byteIndex] += dbl_doubles[j];
    }

    int remainder = j % 2;
    if (remainder == 0) {
      //    if (i == 0 || i == 2 || i == 4 || i == 6 || i == 8 || i == 10 || i == 12 || i == 14) {
      dbl_data[dbl_byteIndex] = dbl_data[dbl_byteIndex] << 4;
    }
    if (remainder == 1) {
      //    if (i == 1 || i 5== 3 || i == 5 || i == 7 || i == 9 || i == 11 || i == 13 || i == 15) {
      dbl_byteIndex++;
    }

  }  // numPlanterRows

  CK_A = 0;

  for (int16_t m = 2; m < dbl_dataSize - 1; m++) {
    CK_A = (CK_A + dbl_data[m]);
  }

  dbl_data[dbl_dataSize - 1] = CK_A;
  SerialPop.write(dbl_data, dbl_dataSize);

  for (int j = 0; j <= numPlanterRows; j++) {
    dbl_doubles[j] = 0;
  }

  for (int k = 5; k < 13; k++) {
    dbl_data[k] = datazero;
  }
}