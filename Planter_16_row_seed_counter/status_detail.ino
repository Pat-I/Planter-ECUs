void statusDetail() {

  for (int row = 0; row < numPlanterRows; row++) {
    unsigned int color = normal_color;  // Par défaut
    if (rc_seedCount[row] < 1) color = red_color;
    else if (rc_skips[row] > 0) color = yellow_color;
    else if (rc_doubles[row] > 0) color = purple_color;

    int dataIndex = 5 + (row / 4);
    int shift = (3 - (row % 4)) * 2;

    if (dataIndex <= 8) {
      rc_data[dataIndex] |= (color << shift);
    }
  }

  rc_data[9] = feedbackCounter++;  // used in AgOpenGPS to tell communications are working

  //add 10 and 11 send sections as active(1) or inactive(0)
  for (uint8_t i = 0; i < numPlanterRows; i++) {
    rc_data[10 + (i >> 3)] |= (!isRowRecoring[i] << (i & 0x07));
  }

  uint8_t ck_a = 0;

  for (int16_t i = 2; i < rc_dataSize - 1; i++) {
    ck_a += rc_data[i];
  }

  rc_data[rc_dataSize - 1] = ck_a;


  SerialPop.write(rc_data, rc_dataSize);
  //    Serial.flush();
  //Serial.println(rc_data[5]);

  for (int j = 0; j <= numPlanterRows; j++) {
    rc_seedCount[j] = 0;
    rc_skips[j] = 0;
    rc_doubles[j] = 0;
  }

  memset(&rc_data[5], 0, 8);

}  // void
