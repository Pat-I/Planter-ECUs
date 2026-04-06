//Stitched together by BabtaiRTK @ 2025
// an attemp to send and recieve AOG PGNs over CANBUS.

// CanDecode(); add to main loop; then decode like serial.
// EncodeAOGtoCAN(): to send the AOGtoCAN[] array over canbus
// Caninit();   add to main Setup  after doSetup()

//id is 3 bytes, 18bits, 19 to 26 are 0s: first (highest) is: 2 for payloads of less than 8 bytes, 1 for std 8 byte AOG sentence, 0 for multiple one, second (middle) byte is source, third (smallest) is destination

//8 bytes or less sentences sent in one message, without CRC
//8 bytes sentence don't send length, 7 and less, the first byte is the payload lenght
//longer sentences are sent in multiple messages, 6 bytes per message, including first byte as the message lenght and the last as CRC
//byte0: nbr/total (4bits / 4bits)
//byte1: sequence nbr (same for the whole sequence)
//byte2 to 7: payload

//#include <ESP32-TWAI-CAN.hpp>

#define CAN_TX 3  // XIAO D6
#define CAN_RX 4  // XIAO D7

//CanFrame SendCan8;
twai_message_t RCV;

void Caninit() {
  // TWAI_MODE_NO_ACK allows the message to send without an external node acknowledging it.
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NO_ACK);

  g_config.tx_queue_len = 64;  // High buffer for AgOpenGPS bursts
  g_config.rx_queue_len = 64;  // High buffer for receiving bursts

  // 2. Timing Config: Set to 250kbps to match your Teensy
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();

  // 3. Filter Config: Accept all incoming messages
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install and Start the driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI Driver Installed");
  } else {
    Serial.println("Failed to install TWAI driver");
    return;
  }

  if (twai_start() == ESP_OK) {
    Serial.println("TWAI Driver Started in NO-ACK Mode");
  } else {
    Serial.println("Failed to start TWAI driver");
    return;
  }
}

void CanDecode() {
  if (twai_receive(&RCV, pdMS_TO_TICKS(0)) == ESP_OK) {  //received a sentence

    CanCheckOldArray();

    //check for an empty byte array
    uint8_t arrayNbr = 17;
    for (uint8_t j = 0; j < 16; j++) {
      if (CANreceiveBuffer[j][0] == 0) {
        arrayNbr = j;
        break;
      }
    }
    if (arrayNbr < 8) {  //we have an empty array
      uint32_t id = RCV.identifier;
      uint8_t idflag = (id >> 16) & 0xFF;
      uint8_t idSrc = (id >> 8) & 0xFF;
      uint8_t idDest = id & 0xFF;

      if (idflag == 1) {                    //standard sentence
        CANreceiveBuffer[arrayNbr][0] = 1;  //this mean there's a sentence to read, must be set to 0 once read
        CANreceiveBuffer[arrayNbr][1] = 0;  //loop counter
        CANreceiveBuffer[arrayNbr][2] = 0;  //sequence counter, not used for single sentences
        CANreceiveBuffer[arrayNbr][3] = idSrc;
        CANreceiveBuffer[arrayNbr][4] = idDest;
        CANreceiveBuffer[arrayNbr][5] = 8;  //data length

        memcpy(&CANreceiveBuffer[arrayNbr][6], RCV.data, 8);
      } else if (idflag == 2) {
        CANreceiveBuffer[arrayNbr][0] = 1;  //this mean there's a sentence to read, must be set to 0 once read
        CANreceiveBuffer[arrayNbr][1] = 0;  //loop counter
        CANreceiveBuffer[arrayNbr][2] = 0;  //sequence counter, not used for single sentences
        CANreceiveBuffer[arrayNbr][3] = idSrc;
        CANreceiveBuffer[arrayNbr][4] = idDest;

        memcpy(&CANreceiveBuffer[arrayNbr][5], RCV.data, 8);
      } else if (idflag == 0) {  //flag is 0, extended AOG PGN over multiple CAN sentences
        //more that 8 bytes payload
        //buf[0] -> message number of the serie
        //buf[1] is a sequence nbr
        //buf[2] of the first message is the number of data bytes
        //so first will contain a payload of 5 bytes, all others contain 6 bytes. the last byte will be the AOG CRC

        uint8_t messageNbr = RCV.data[0];
        uint8_t sequenceNbr = RCV.data[1];

        if (messageNbr == 1 && arrayNbr < 8) {  //new message
          //write the message
          CANreceiveBuffer[arrayNbr][0] = 2;            //this mean we are writing a longer PGN
          CANreceiveBuffer[arrayNbr][1] = 0;            //loop counter
          CANreceiveBuffer[arrayNbr][2] = sequenceNbr;  //sequence nbr
          CANreceiveBuffer[arrayNbr][3] = idSrc;
          CANreceiveBuffer[arrayNbr][4] = idDest;

          // include the length in buf2
          memcpy(&CANreceiveBuffer[arrayNbr][5], &RCV.data[2], 6);
        } else {  //continue an existing one
          for (uint8_t k = 0; k < 8; k++) {
            if (messageNbr == CANreceiveBuffer[k][0] && sequenceNbr == CANreceiveBuffer[k][2] && idSrc == CANreceiveBuffer[k][3] && idDest == CANreceiveBuffer[k][4]) {
              //It's the next message
              uint8_t messageTotal = ((CANreceiveBuffer[k][5] + 6) / 6);
              if (messageNbr < messageTotal) {
                CANreceiveBuffer[k][0] = messageNbr + 1;
              } else {
                CANreceiveBuffer[k][0] = 1;  //last part, read to read
              }
              CANreceiveBuffer[k][1] = 0;  // reset loop counter

              memcpy(&CANreceiveBuffer[k][messageNbr * 6 - 1], &RCV.data[2], 6);
              break;
            }
          }
        }
      }
    }
  }
  //else Serial.println("No free array");
}

void CanCheckOldArray() {
  //should be run at 10 to 1000hz
  for (uint8_t k = 0; k < 8; k++) {
    if (CANreceiveBuffer[k][0] > 0) {
      CANreceiveBuffer[k][1]++;
      if (CANreceiveBuffer[k][1] > 250) {
        /*
        Serial.print("Sec= ");
        Serial.print(CANreceiveBuffer[k][0]);
        Serial.print(" , Source= ");
        Serial.print(CANreceiveBuffer[k][3]);
        Serial.print(" , PGN= ");
        Serial.println(CANreceiveBuffer[k][4]);
*/
        CANreceiveBuffer[k][0] = 0;  // array erased
      }
    }
  }
}

void EncodeAOGtoCAN(const uint8_t* data, uint8_t dataLen) {
  // data[2] = src, data[3] = dest, data[4] = length
  if (dataLen > 4 && data[2] > 0 && data[3] > 0) {
    uint8_t src = data[2];
    uint8_t dest = data[3];
    uint8_t leng = data[4];

    if (leng == 8) {
      CanEncode(1, src, dest, &data[5]);
    } else if (leng < 8) {
      CanEncode(2, src, dest, &data[4]);
    } else {
      AOGtoCANseq++;
      uint8_t numMsgs = (leng + 6) / 6;

      // --- Premier message ---
      uint8_t firstBuf[8] = { 1, AOGtoCANseq, leng, 0, 0, 0, 0, 0 };
      memcpy(&firstBuf[3], &data[5], 5);
      CanEncode(0, src, dest, firstBuf);

      // --- Messages suivants ---
      const uint8_t* dataPtr = &data[10];
      for (uint8_t i = 1; i < numMsgs; i++) {
        uint8_t nextBuf[8] = { (uint8_t)(i + 1), AOGtoCANseq, 0, 0, 0, 0, 0, 0 };
        memcpy(&nextBuf[2], dataPtr, 6);
        CanEncode(0, src, dest, nextBuf);
        dataPtr += 6;
      }
    }
  }
}

inline void CanEncode(uint8_t flag, uint8_t src, uint8_t dest, const uint8_t* dPtr) {
  twai_message_t message = { 0 };
  uint32_t id = (dest & 0xFF) | ((src & 0xFF) << 8) | ((flag & 0xFF) << 16);
  message.identifier = id;
  message.extd = 1;
  message.rtr = 0;
  message.data_length_code = 8;

  // Copie les 8 octets d'un coup dans le tableau 'data' de la structure TWAI
  memcpy(message.data, dPtr, 8);

  // Transmet avec un timeout très court (1ms) pour ne pas bloquer la loop
  twai_transmit(&message, pdMS_TO_TICKS(1));
}