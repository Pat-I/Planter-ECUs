//Stitched together by BabtaiRTK @ 2025
// an attemp to send and recieve AOG PGNs over CANBUS.

// CanDecode(); add to main loop; then decode like serial.
// EncodeAOGtoCAN(): to send the AOGtoCAN[] array over canbus
// Caninit();   add to main Setup  after doSetup()

//id is 3 bytes, 18bits, 19 to 26 are 0s: first (highest) is:
//third bit: 1 for internal, 0 for sending to AOG, so: 5 is for internal 8 bytes sentence
//first two bits: 2 for payloads of less than 8 bytes, 1 for std 8 byte AOG sentence, 0 for multiple one, second (middle) byte is source, third (smallest) is destination

//8 bytes or less sentences sent in one message, without CRC
//8 bytes sentence don't send length, 7 and less, the first byte is the payload lenght
//longer sentences are sent in multiple messages, 6 bytes per message, including first byte as the message lenght and the last as CRC
//byte0: nbr/total (4bits / 4bits)
//byte1: sequence nbr (same for the whole sequence)
//byte2 to 7: payload

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_128> CanMain;  //first slot is can3

CAN_message_t SendCan8;
CAN_message_t RCV;

void Caninit() {
  CanMain.begin();
  CanMain.setBaudRate(250000);
  SendCan8.flags.extended = 1;
  SendCan8.len = 8;
}

void CanDecode() {
  if (CanMain.read(RCV)) {
    //check for an empty byte array
    uint8_t arrayNbr = 17;
    for (uint8_t j = 0; j < 16; j++) {
      if (CANreceiveBuffer[j][0] == 0) {
        arrayNbr = j;
        break;
      }
    }
    //CanCheckOldArray();
    if (arrayNbr < 16) {  //we have an empty array
      uint32_t id = RCV.id;
      uint8_t idflag = (id >> 16) & 0xFF;  //check all 8 bits
      uint8_t idSrc = (id >> 8) & 0xFF;
      uint8_t idDest = id & 0xFF;

      if (idflag == 1 || idflag == 5) {     //standard sentence
        CANreceiveBuffer[arrayNbr][0] = 1;  //this mean there's a sentence to read, must be set to 0 once read
        CANreceiveBuffer[arrayNbr][1] = 0;  //loop counter
        CANreceiveBuffer[arrayNbr][2] = 0;  //sequence counter, not used for single sentences
        CANreceiveBuffer[arrayNbr][3] = idSrc;
        CANreceiveBuffer[arrayNbr][4] = idDest;
        CANreceiveBuffer[arrayNbr][5] = 8;  //data length

        memcpy(&CANreceiveBuffer[arrayNbr][6], RCV.buf, 8);
      } else if (idflag == 2 || idflag == 6) {
        CANreceiveBuffer[arrayNbr][0] = 1;  //this mean there's a sentence to read, must be set to 0 once read
        CANreceiveBuffer[arrayNbr][1] = 0;  //loop counter
        CANreceiveBuffer[arrayNbr][2] = 0;  //sequence counter, not used for single sentences
        CANreceiveBuffer[arrayNbr][3] = idSrc;
        CANreceiveBuffer[arrayNbr][4] = idDest;

        memcpy(&CANreceiveBuffer[arrayNbr][5], RCV.buf, 8);
      } else if (idflag == 0 || idflag == 4) {  //flag is 0, extended AOG PGN over multiple CAN sentences

        //more that 8 bytes payload
        //buf[0] message number of the serie
        //buf[1] is a sequence nbr
        //buf[2] of the first message is the number of data bytes
        //so first will contain a payload of 5 bytes, all others contain 6 bytes. the last byte will be the AOG CRC
        uint8_t messageNbr = RCV.buf[0];
        uint8_t sequenceNbr = RCV.buf[1];

        if (messageNbr == 1 && arrayNbr < 8) {  //new message
          //write the message
          CANreceiveBuffer[arrayNbr][0] = 2;            //this mean we are writing a longer PGN
          CANreceiveBuffer[arrayNbr][1] = 0;            //loop counter
          CANreceiveBuffer[arrayNbr][2] = sequenceNbr;  //sequence nbr
          CANreceiveBuffer[arrayNbr][3] = idSrc;
          CANreceiveBuffer[arrayNbr][4] = idDest;
          // include the length in buf2
          memcpy(&CANreceiveBuffer[arrayNbr][5], &RCV.buf[2], 6);
        } else {  //continue an existing one
          for (uint8_t k = 0; k < 8; k++) {
            if (sequenceNbr == CANreceiveBuffer[k][2] && idSrc == CANreceiveBuffer[k][3] && idDest == CANreceiveBuffer[k][4]) {
              //It's the next message
              uint8_t messageTotal = ((CANreceiveBuffer[k][5] + 6) / 6);

              if (CANreceiveBuffer[k][0] < messageTotal) {
                CANreceiveBuffer[k][0] += 1;
              } else {
                CANreceiveBuffer[k][0] = 1;  //last part, read to read
              }
              CANreceiveBuffer[k][1] = 0;  // reset loop counter

              memcpy(&CANreceiveBuffer[k][messageNbr * 6 - 1], &RCV.buf[2], 6);
              break;
            }
          }
        }
      }
    }
  }
}

void CanCheckOldArray() {
  //should be run at 10 to 1000hz
  for (uint8_t k = 0; k < 8; k++) {
    if (CANreceiveBuffer[k][0] > 1) {
      CANreceiveBuffer[k][1]++;
      if (CANreceiveBuffer[k][1] > 250) CANreceiveBuffer[k][0] = 0;  // array erased
    }
  }
}

void EncodeAOGtoCAN(const uint8_t* data, uint8_t dataLen, bool isSentToAOG) {

  // Basic validation based on your protocol (Source at index 2, Destination at index 3)
  if (dataLen > 4 && data[2] > 0 && data[3] > 0) {

    uint8_t src = data[2];
    uint8_t dest = data[3];
    uint8_t leng = data[4];
    uint8_t sendTo = isSentToAOG ? 0 : 4;

    if (leng == 8) {
      // Single 8-byte sentence: start data from index 5
      CanEncode(sendTo + 1, src, dest, &data[5]);
    } else if (leng < 8) {
      // Single short sentence: start data from index 4
      CanEncode(sendTo + 2, src, dest, &data[4]);
    } else {
      // MULTIPLE SENTENCES
      AOGtoCANseq++;
      uint8_t numMsgs = (leng + 6) / 6;
      // --- Handle First Message ---
      uint8_t msgIndex = 1;
      uint8_t firstBuf[8];
      firstBuf[0] = msgIndex;
      firstBuf[1] = AOGtoCANseq;
      // Copy index 4 (lenght) and 5 bytes of actual data from index 5 to 9
      memcpy(&firstBuf[2], &data[4], 6);

      CanEncode(sendTo, src, dest, firstBuf);

      // --- Handle Subsequent Messages (Pointer Arithmetic) ---
      // Point to the next available data (AOGtoCAN[10])
      const uint8_t* dataPtr = &data[10];

      for (uint8_t i = 1; i < numMsgs; i++) {
        msgIndex = i + 1;

        uint8_t nextBuf[8];
        nextBuf[0] = msgIndex;
        nextBuf[1] = AOGtoCANseq;
        // Copy 6 bytes of data into the CAN buffer
        memcpy(&nextBuf[2], dataPtr, 6);

        CanEncode(sendTo, src, dest, nextBuf);
        // Advance pointer by 6 bytes for the next iteration (avoids multiplication)
        dataPtr += 6;
      }
    }
  }
}

inline void CanEncode(uint8_t flag, uint8_t src, uint8_t dest, const uint8_t* dPtr) {
  // Build ID: (Flag << 16) | (Source << 8) | Destination
  SendCan8.id = (dest & 0xFF) | ((src & 0xFF) << 8) | ((flag & 0xFF) << 16);

  memcpy(SendCan8.buf, dPtr, 8);

  CanMain.write(SendCan8);
}