

char arduinoDate[] = "2026-02-26";
char firmwareName[] = "JD1770NT main machine ECU";
char arduinoVersion[] = "v 1.0.0";

/*  PWM Frequency -> 
   *   490hz (default) = 0
   *   122hz = 1
   *   3921hz = 2
   */
#define PWM_Frequency 1

//loop time variables in milliseconds
const byte LOOP_TIME = 100;  // 10Hz
unsigned long lastTime = LOOP_TIME;
unsigned long currentTime = LOOP_TIME;

//communication
uint8_t CANreceiveBuffer[255];
uint8_t CANreceiveErrorCnt = 0;
//define inputs and outputs
#define PWM1_CYTRON 3
#define DIR1_CYTRON 4

#define BOUTON_UP 6
#define BOUTON_DOWN 7

#define POTO_UP A0
#define POTO_DOWN A1

void setup() {
  //PWM rate settings. Set them both the same!!!!
  /*  PWM Frequency ->
       490hz (default) = 0
       122hz = 1
       3921hz = 2
  */
  if (PWM_Frequency == 0) {
    //analogWriteFrequency(PWM1_LPWM, 490);
    //analogWriteFrequency(PWM2_RPWM, 490);
  } else if (PWM_Frequency == 1) {
    //analogWriteFrequency(PWM1_LPWM, 122);
    //analogWriteFrequency(PWM2_RPWM, 122);
  } else if (PWM_Frequency == 2) {
    //analogWriteFrequency(PWM1_LPWM, 3921);
    //analogWriteFrequency(PWM2_RPWM, 3921);
  }
  Serial.begin(115200);
  //pinMode is only for digital pins?
  pinMode(BOUTON_UP, INPUT);  //INSTEAD INPUT_PULLUP, not needed?
  pinMode(BOUTON_DOWN, INPUT);
  pinMode(DIR1_CYTRON, OUTPUT);






  delay(100);
  Serial.println(firmwareName);
  Serial.println(arduinoVersion);
  Serial.println(arduinoDate);




  Caninit();
}

void loop() {
  // put your main code here, to run repeatedly:

  //Loop triggers every 100 msec
  currentTime = millis();

  if (currentTime - lastTime >= LOOP_TIME) {
    lastTime = currentTime;
    analogRead(BOUTON_UP);
    analogWrite(PWM1_CYTRON, 128);







  }  // end of 100 ms loop


  CanDecode();
  //to add: read the CANreceiveBuffer
  CheckDataFromCAN();
}  // end of loop

void CheckDataFromCAN() {
  if (CANreceiveBuffer[0] == 1) {
    CANreceiveBuffer[0] = 0;
    switch (CANreceiveBuffer[1]) {
      case 123:  //planter monitor, just forward


        break;

      case 127:  //7F, from AOG

        break;
    }
  }
}
