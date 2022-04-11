// Arduino Joystick with FFB library from here:
//   https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary
#include <Joystick.h>


#define DEBUG false

#define ENCODER_MAX_VALUE 2100
#define ENCODER_MIN_VALUE -2100
#define ENCODER_OUT_A_PIN 2
#define ENCODER_OUT_B_PIN 3

#define BTS7960_RPWM_PIN 5
#define BTS7960_LPWM_PIN 6
#define BTS7960_R_EN_PIN 8
#define BTS7960_L_EN_PIN 9

#define AUTO_SEND_STATE false



Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID, // hidReportId
  JOYSTICK_TYPE_JOYSTICK, // joystickType
  0, // buttonCount
  0, // hatSwitchCount
  false, // includeXAxis
  false, // includeYAxis 
  false, // includeZAxis
  false, // includeRxAxis
  false, // includeRyAxis
  false, // includeRzAxis
  false, // includeRudder
  false, // includeThrottle
  false, // includeAccelerator
  false, // includeBrake
  true  // includeSteering
); 



//------------------------------------------
//Note: not every pin is usable for interrupts!
//You need to connect the signal wires to pins which are usable for interrupts: Micro, Leonardo, other 32u4-based: 0, 1, 2, 3, 7
volatile int32_t encoder_state = 0; // volatile is using RAM this is because we will use interrupts
int32_t encoder_previous_state = 0;

// FFB
Gains mygains[2];
EffectParams myeffectparams[2];
int32_t forces[2] = {0};



void encoderOutAChange() {
  // when outA changes, outA==outB means negative direction
  encoder_state += digitalRead(ENCODER_OUT_A_PIN) == digitalRead(ENCODER_OUT_B_PIN) ? -1 : 1; 
}


void encoderOutBChange() {
  // when outB changes, outA==outB means positive direction
  encoder_state += digitalRead(ENCODER_OUT_A_PIN) == digitalRead(ENCODER_OUT_B_PIN) ? 1 : -1;
}



void setup() {
  #if DEBUG
    Serial.begin(9600); 
  #endif
  
  Joystick.begin(AUTO_SEND_STATE);

  // ENCODER
  Joystick.setSteeringRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);

  pinMode(ENCODER_OUT_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_OUT_B_PIN, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_OUT_A_PIN),
    encoder_out_A_change,
    CHANGE);
  attachInterrupt(
    digitalPinToInterrupt(ENCODER_OUT_B_PIN),
    encoder_out_B_change,
    CHANGE);
  // ENCODER

  // BTS7960 MOTOR DRIVER
  pinMode(BTS7960_RPWM_PIN, OUTPUT);
  pinMode(BTS7960_LPWM_PIN, OUTPUT);
  pinMode(BTS7960_R_EN_PIN, INPUT);
  pinMode(BTS7960_L_EN_PIN, INPUT);
  // BTS7960 MOTOR DRIVER

}


void loop() {
  // ENCODER & STEERING AXIS
  if (encoder_previous_state != encoder_state) {
    if(encoder_state < ENCODER_MIN_VALUE){
      encoder_state = ENCODER_MIN_VALUE;
    }
    else if(encoder_state > ENCODER_MAX_VALUE){
      encoder_state = ENCODER_MAX_VALUE;
    }
    encoder_previous_state = encoder_state; 
    Joystick.setSteering(encoder_state);
    #if DEBUG
      Serial.println(encoder_state);
    #endif
  }
  // ENCODER & STEERING AXIS
  

  #if !AUTO_SEND_STATE
    Joystick.sendState();
  #endif

  // FORCE FEEDBACK
  // Joystick.setEffectParams(myeffectparams);
  Joystick.getForce(forces);
  if(forces[0] > 0){
    // digitalWrite(6,LOW);
    // digitalWrite(7,HIGH);
    analogWrite(9,abs(forces[0]));
  }else{
    // digitalWrite(6,HIGH);
    // digitalWrite(7,LOW);
    analogWrite(9,abs(forces[0]));
  }
  // FORCE FEEDBACK

  delay(1);
}
