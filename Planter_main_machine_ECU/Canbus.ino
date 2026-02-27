//Stitched together by BabtaiRTK @ 2025
// an attemp to send and recieve AOG PGNs over CANBUS.

// CanDecode(); add to main loop  comment out  ReceiveUDP(); and ReceiveUpdate();
// CanEncode(); add to main loop  comment out  SendComm();
// Caninit();   add to main Setup  after doSetup()

//id is 3 bytes: first (highest) is 1 for std 8 byte AOG sentence, 0 for multiple one, second (middle) byte is source, third (smallest) is destination

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CanMain;  //first slot is can3

CAN_message_t SendCan8;
CAN_message_t RCV;

void Caninit() {
  CanMain.begin();
  CanMain.setBaudRate(250000);
  SendCan8.flags.extended = 1;
  SendCan8.len = 8;
}

void CanDecode() {
  if (CANreceiveBuffer[0] != 1) {  // no sentence pending reading

    if (CanMain.read(RCV)) {  //received a sentence

      uint32_t id = RCV.id;
      uint8_t idflag = (id >> 16) & 0x01;
      uint8_t idSrc = (id >> 8) & 0xFF;
      uint8_t idDest = id & 0xFF;

      if (idflag == 1) {          //standard sentence
      if(CANreceiveBuffer[0] > 1) CANreceiveErrorCnt++; //we overwrite some mutiple can frames message
        CANreceiveBuffer[0] = 1;  //this mean there's a sentence to read, must be set to 0 once read
        CANreceiveBuffer[1] = idSrc;
        CANreceiveBuffer[2] = idDest;
        CANreceiveBuffer[3] = 8;  //data length
        for (uint8_t i = 0; i < 8; i++) {
          CANreceiveBuffer[i + 4] = RCV.buf[i];
        }
      } else {  //flag is 0, extended AOG PGN over multiple CAN sentences
        //more that 8 bytes payload
        //buf[0] -> 4bytes message number and 4 bytes number of messages for all sentences
        //buf[1] of the first message is the number of data bytes
        //so first will contain a payload of 6 bytes, all others contain 7 bytes.

        uint8_t messageNbr = (RCV.buf[0] >> 4) & 0x0F;
        uint8_t messageTotal = RCV.buf[0] & 0x0F;

        if (messageNbr == 1) {
          if (CANreceiveBuffer[0] > 1) {
            // there's already one writing, bad!
            CANreceiveErrorCnt++;
          } else {
            //write the message
            CANreceiveBuffer[0] = 2;  //this mean we are writing a longer PGN
            CANreceiveBuffer[1] = idSrc;
            CANreceiveBuffer[2] = idDest;
            CANreceiveBuffer[3] = RCV.buf[1];  //data length
            for (uint8_t i = 2; i < 8; i++) {
              CANreceiveBuffer[i + 2] = RCV.buf[i];
            }
          }
        } else {
          if (messageNbr == CANreceiveBuffer[0] && idSrc == CANreceiveBuffer[1] && idDest == CANreceiveBuffer[2]) {
            //It's the next message
            if (messageNbr < messageTotal){
              CANreceiveBuffer[0] = messageNbr + 1;
            } else {
              CANreceiveBuffer[0] = 1; //last part, read to read
            }
            for (uint8_t i = 1; i < 8; i++) {
              CANreceiveBuffer[messageNbr * 7 + i - 5] = RCV.buf[i];
            }
          } else {
            //error! It's an other sentence
            CANreceiveErrorCnt++;
          }
        }
      }
    }
  }
}

void CanEncode(uint8_t src, uint8_t dest, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7) {

  //uint32_t id = dest | src << 8 | 1 << 16;
  uint32_t id = (dest & 0xFF) | ((src & 0xFF) << 8) | (1 << 16);
  SendCan8.buf[0] = data0;
  SendCan8.buf[1] = data1;
  SendCan8.buf[2] = data2;
  SendCan8.buf[3] = data3;
  SendCan8.buf[4] = data4;
  SendCan8.buf[5] = data5;
  SendCan8.buf[6] = data6;
  SendCan8.buf[7] = data7;

  SendCan8.id = id;
  CanMain.write(SendCan8);
}