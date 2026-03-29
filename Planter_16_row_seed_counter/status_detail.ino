void statusDetail() {
  int numToTest = 4;
  if (numPlanterRows < 4) numToTest = numPlanterRows;
  for (int i = numToTest - 1; i >= 0; i--) {

    if (isRowRecoring[i] && rc_seedCount[i] < 1) {
      rc_data[5] += red_color;
      //            Serial.println("red 01");
    } else if (rc_skips[i] > 0) {
      rc_data[5] += yellow_color;
      //              Serial.print(i);
      //              Serial.print("  ");
      //                    Serial.println("yellow");
    } else if (rc_doubles[i] > 0) {
      rc_data[5] += purple_color;
      //              Serial.print(i);
      //              Serial.println(" purple");
    } else {
      rc_data[5] += normal_color;
      //            Serial.println("normal 00");
    }
    if (i > 0) rc_data[5] = rc_data[5] << 2;
  }

  numToTest = 8;
  if (numPlanterRows < 8) numToTest = numPlanterRows;
  for (int i = numToTest - 1; i >= 4; i--) {
    if (isRowRecoring[i] && rc_seedCount[i] < 1) {
      rc_data[6] += red_color;
      //      Serial.println("red 01");
      if (i > 4) rc_data[6] = rc_data[6] << 2;
    } else if (rc_skips[i] > 0) {
      rc_data[6] += yellow_color;
      //      Serial.println("yellow 10");
      if (i > 4) rc_data[6] = rc_data[6] << 2;
    } else if (rc_doubles[i] > 0) {
      rc_data[6] += purple_color;
      //              Serial.print(i);
      //              Serial.println(" purple");
      if (i > 4) rc_data[6] = rc_data[6] << 2;
    } else {
      rc_data[6] += normal_color;
      //      Serial.println("normal 00");
      if (i > 4) rc_data[6] = rc_data[6] << 2;
    }
  }
  numToTest = 12;
  if (numPlanterRows < 12) numToTest = numPlanterRows;
  for (int i = numToTest - 1; i >= 8; i--) {
    if (isRowRecoring[i] && rc_seedCount[i] < 1) {
      rc_data[7] += red_color;
      //      Serial.println("red 01");
      if (i > 8) rc_data[7] = rc_data[7] << 2;
    } else if (rc_skips[i] > 0) {
      rc_data[7] += yellow_color;
      //Serial.println("yellow 10");
      if (i > 8) rc_data[7] = rc_data[7] << 2;
    } else if (rc_doubles[i] > 0) {
      rc_data[7] += purple_color;
      //      Serial.println("purple 11");
      if (i > 8) rc_data[7] = rc_data[7] << 2;
    } else {
      rc_data[7] += normal_color;
      //    Serial.println("normal 00");
      if (i > 8) rc_data[7] = rc_data[7] << 2;
    }
  }
  numToTest = 16;
  if (numPlanterRows < 16) numToTest = numPlanterRows;
  for (int i = numToTest - 1; i >= 12; i--) {
    //            Serial.print(i);
    if (isRowRecoring[i] && rc_seedCount[i] < 1) {
      rc_data[8] += red_color;
      //              Serial.println("red 01");
      if (i > 12) rc_data[8] = rc_data[8] << 2;
    } else if (rc_skips[i] > 0) {
      rc_data[8] += yellow_color;
      //                      Serial.println(" yellow 10");
      if (i > 12) rc_data[8] = rc_data[8] << 2;
    } else if (rc_doubles[i] > 0) {
      rc_data[8] += purple_color;
      //                      Serial.println(" purple 11");
      if (i > 12) rc_data[8] = rc_data[8] << 2;
    } else {
      rc_data[8] += normal_color;
      //            Serial.println("normal 00");
      if (i > 12) rc_data[8] = rc_data[8] << 2;
    }
  }

  rc_data[9] = feedbackCounter++;  // used in AgOpenGPS to tell communications are working

  int16_t CK_A = 0;

  for (int16_t i = 2; i < rc_dataSize - 1; i++) {
    CK_A = (CK_A + rc_data[i]);
  }

  rc_data[rc_dataSize - 1] = CK_A;


  Serial1.write(rc_data, rc_dataSize);
  //    Serial.flush();
  Serial.println(rc_data[5]);

  for (int j = 0; j <= numPlanterRows; j++) {
    rc_seedCount[j] = 0;
    rc_skips[j] = 0;
    rc_doubles[j] = 0;
  }

  rc_data[5] = datazero;
  rc_data[6] = datazero;
  rc_data[7] = datazero;
  rc_data[8] = datazero;

}  // void
