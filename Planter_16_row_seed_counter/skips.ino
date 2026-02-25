void skipdetail() {

  sk_byteIndex = 5;

  for (int i = 0; i < numPlanterRows; i++) {
    if (sk_skips[i] > 0) {
      (sk_skips[i] > 7) ? sk_data[sk_byteIndex] = 7 : sk_data[sk_byteIndex] += sk_skips[i];
    }

    int remainder = i % 2;
    if (remainder == 0) {
      sk_data[sk_byteIndex] = sk_data[sk_byteIndex] << 4;
    }
    if (remainder == 1) {
      sk_byteIndex++;
    }
  }  // numPlanterRows

  int16_t CK_A = 0;

  for (int16_t i = 2; i < sk_dataSize - 1; i++) {
    CK_A = (CK_A + sk_data[i]);
  }

  sk_data[sk_dataSize - 1] = CK_A;
  Serial1.write(sk_data, sk_dataSize);

  for (int j = 0; j <= numPlanterRows; j++) {
    sk_skips[j] = 0;
  }

  for (int k = 5; k < 13; k++) {
    sk_data[k] = datazero;
  }
}  // void
