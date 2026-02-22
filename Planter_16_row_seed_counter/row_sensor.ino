void RetrieveRowData()
{


  for(uint8_t i = 0; i< 16; i++)
  {
    if(sensorNewData[i])
    {
      sensorNewData[i] = 0; // ready for next time
      noInterrupts()
      sensorSeedTimeStable[i] = sensorSeedTime[i];
      interrupts()

      sensorSeedDuration[i] = sensorSeedTimeStable[i] - lastSensorTime[i];
      lastSensorTime[i] = sensorSeedTimeStable[i];
      if(sensorSeedDuration[i] < 500 && sensorSeedDuration[i] > 0)// valid data
      {
        
        
        if(!isRowSeeding[i])
        {
          if(sensorAllTimes[i][0] > 0)
          {
           // two seeds in a row
            isRowSeeding[i] = true;
          }
        }
        
        
      }
      else 
      {
      // too long, no seeding or under 0 no valid data
       sensorSeedDuration[i] = 0;
       //isRowSeeding[i] = false;
      }

      //add the data to the array
      for(uint8_t j = 99; j >= 0; j--)
      {
        sensorAllTimes[i][j] = sensorAllTimes[i][j - 1];
      }
      sensorAllTimes[i][0] = sensorSeedDuration[i];
    }
  }

}