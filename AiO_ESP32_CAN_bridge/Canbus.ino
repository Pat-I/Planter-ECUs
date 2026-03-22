//Stitched together by BabtaiRTK @ 2025
// an attemp to send and recieve AOG PGNs over CANBUS.

// CanDecode(); add to main loop; then decode like serial.
// EncodeAOGtoCAN(): to send the AOGtoCAN[] array over canbus
// Caninit();   add to main Setup  after doSetup()

//id is 3 bytes: first (highest) is 1 for std 8 byte AOG sentence, 0 for multiple one, second (middle) byte is source, third (smallest) is destination

//8 bytes sentences sent in one message, without CRC
//longer sentences are sent in multiple messages, 6 bytes per message, including first byte as the message lenght and the last as CRC
//byte0: nbr/total (4bits / 4bits)
//byte1: sequence nbr (same for the whole sequence)
//byte2 to 7: payload

#include <ESP32-TWAI-CAN.hpp>

#define CAN_TX 2  // XIAO D6
#define CAN_RX 3  // XIAO D7

//CanFrame SendCan8;
CanFrame RCV;

void Caninit() {
  ESP32Can.setPins(CAN_TX, CAN_RX);

  // You can set custom size for the queues - those are default
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);

  // .setSpeed() and .begin() functions require to use TwaiSpeed enum,
  // but you can easily convert it from numerical value using .convertSpeed()
  ESP32Can.setSpeed(ESP32Can.convertSpeed(250));

  // You can also just use .begin()..
  if (ESP32Can.begin()) {
    Serial.println("CAN bus started!");
  } else {
    Serial.println("CAN bus failed!");
  }
}

void CanDecode() {
  //check for an empty byte array
  uint8_t arrayNbr = 15;
  for (uint8_t j = 0; j < 8; j++) {
    if (CANreceiveBuffer[j][0] == 0) {
      arrayNbr = j;
      break;
    }
  }
  if (arrayNbr < 8) {                  //we have an empty array
    if (ESP32Can.readFrame(RCV, 1)) {  //received a sentence
      uint32_t id = RCV.identifier;
      uint8_t idflag = (id >> 16) & 0x01;
      uint8_t idSrc = (id >> 8) & 0xFF;
      uint8_t idDest = id & 0xFF;
      Serial.print("PGN ");
      Serial.println(idDest);

      if (idflag == 1) {                    //standard sentence
        CANreceiveBuffer[arrayNbr][0] = 1;  //this mean there's a sentence to read, must be set to 0 once read
        CANreceiveBuffer[arrayNbr][1] = 0;  //loop counter
        CANreceiveBuffer[arrayNbr][2] = 0;  //sequence counter, not used for single sentences
        CANreceiveBuffer[arrayNbr][3] = idSrc;
        CANreceiveBuffer[arrayNbr][4] = idDest;
        CANreceiveBuffer[arrayNbr][5] = 8;  //data length
        for (uint8_t i = 0; i < 8; i++) {
          CANreceiveBuffer[arrayNbr][i + 6] = RCV.data[i];
        }
      } else {  //flag is 0, extended AOG PGN over multiple CAN sentences
        //more that 8 bytes payload
        //buf[0] -> 4bytes message number and 4 bytes number of messages for all sentences
        //buf[1] is a sequence nbr
        //buf[2] of the first message is the number of data bytes
        //so first will contain a payload of 5 bytes, all others contain 6 bytes. the last byte will be the AOG CRC

        uint8_t messageNbr = (RCV.data[0] >> 4) & 0x0F;
        uint8_t messageTotal = RCV.data[0] & 0x0F;
        uint8_t sequenceNbr = RCV.data[1];

        if (messageNbr == 1) {  //new message
          //write the message
          CANreceiveBuffer[arrayNbr][0] = 2;            //this mean we are writing a longer PGN
          CANreceiveBuffer[arrayNbr][1] = 0;            //loop counter
          CANreceiveBuffer[arrayNbr][2] = sequenceNbr;  //sequence nbr
          CANreceiveBuffer[arrayNbr][3] = idSrc;
          CANreceiveBuffer[arrayNbr][4] = idDest;
          for (uint8_t i = 2; i < 8; i++) {
            // include the length in buf2
            CANreceiveBuffer[arrayNbr][i + 3] = RCV.data[i];
          }
        } else {  //continue an existing one
          for (uint8_t k = 0; k < 8; k++) {
            if (messageNbr == CANreceiveBuffer[k][0] && sequenceNbr == CANreceiveBuffer[k][2] && idSrc == CANreceiveBuffer[k][3] && idDest == CANreceiveBuffer[k][4]) {
              //It's the next message
              if (messageNbr < messageTotal) {
                CANreceiveBuffer[k][0] = messageNbr + 1;
              } else {
                CANreceiveBuffer[k][0] = 1;  //last part, read to read
              }
              CANreceiveBuffer[k][1] = 0;  // reset loop counter
              for (uint8_t i = 2; i < 8; i++) {
                CANreceiveBuffer[k][messageNbr * 6 + i - 3] = RCV.data[i];
              }
            }
          }
        }
      }
    }
  }
  else Serial.println("No free array");
}

void CanCheckOldArray() {
  //should be run at 10 to 1000hz
  for (uint8_t k = 0; k < 8; k++) {
    if (CANreceiveBuffer[k][0] > 0) {
      CANreceiveBuffer[k][1]++;
      if (CANreceiveBuffer[k][1] > 25) CANreceiveBuffer[k][0] = 0;  // array erased
    }
  }
}

void EncodeAOGtoCAN() {
  //Input format: 0x80, 0x81, source, dest, lenght, data ........, CRC
  if (AOGtoCAN[2] > 0 && AOGtoCAN[3] > 0) {  //something to send
    uint8_t leng = min(AOGtoCAN[4], (uint8_t)245);
    if (leng <= 8) {  //single sentence
      CanEncode(1, AOGtoCAN[2], AOGtoCAN[3], AOGtoCAN[5], AOGtoCAN[6], AOGtoCAN[7], AOGtoCAN[8], AOGtoCAN[9], AOGtoCAN[10], AOGtoCAN[11], AOGtoCAN[12]);
    } else {  //multiple sentences
      AOGtoCANseq++;
      uint8_t NumberOfMessages = (leng + 1) / 6;
      uint8_t messageNumber = (NumberOfMessages & 0x0F0) | ((1 & 0x0F) << 4);
      //first message
      //flag, source, dest, nbr/total, sequence, lenght, data 0-4
      CanEncode(0, AOGtoCAN[2], AOGtoCAN[3], messageNumber, AOGtoCANseq, leng, AOGtoCAN[5], AOGtoCAN[6], AOGtoCAN[7], AOGtoCAN[8], AOGtoCAN[9]);
      for (uint8_t i = 1; i < NumberOfMessages; i++) {
        messageNumber = (NumberOfMessages & 0x0F0) | (((i + 1) & 0x0F) << 4);
        CanEncode(0, AOGtoCAN[2], AOGtoCAN[3], messageNumber, AOGtoCANseq, AOGtoCAN[i * 6 + 4], AOGtoCAN[i * 6 + 5], AOGtoCAN[i * 6 + 6], AOGtoCAN[i * 6 + 7], AOGtoCAN[i * 6 + 8], AOGtoCAN[i * 6 + 9]);
      }
    }
    for (uint8_t j = 2; j < (leng + 6); j++) {
      AOGtoCAN[j] = 0;
    }
  }
}

void CanEncode(uint8_t flag, uint8_t src, uint8_t dest, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7) {

  //uint32_t id = dest | src << 8 | 1 << 16;
  CanFrame SendCan8  = { 0 };
  uint32_t id = (dest & 0xFF) | ((src & 0xFF) << 8) | ((flag & 1) << 16);
  SendCan8.identifier = id;
  SendCan8.extd = 1;
  SendCan8.data_length_code = 8;

  SendCan8.data[0] = data0;
  SendCan8.data[1] = data1;
  SendCan8.data[2] = data2;
  SendCan8.data[3] = data3;
  SendCan8.data[4] = data4;
  SendCan8.data[5] = data5;
  SendCan8.data[6] = data6;
  SendCan8.data[7] = data7;

  ESP32Can.writeFrame(SendCan8, 1);
}