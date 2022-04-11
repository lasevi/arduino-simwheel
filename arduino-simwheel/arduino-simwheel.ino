// Arduino Joystick with FFB library from here:
//   https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary
#include <Joystick.h>


#define DEBUG true
#define AUTO_SEND_STATE false

#define ENCODER_OUT_A_PIN 2
#define ENCODER_OUT_B_PIN 3
#define ENCODER_MAX_VALUE 2100
#define ENCODER_MIN_VALUE -2100
#define STEERING_AXIS_MIN_VALUE -512
#define STEERING_AXIS_MAX_VALUE 511

#define BTS7960_RPWM_PIN 5
#define BTS7960_LPWM_PIN 6
#define BTS7960_R_EN_PIN 8
#define BTS7960_L_EN_PIN 9




Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID, // hidReportId
  JOYSTICK_TYPE_JOYSTICK, // joystickType
  0, // buttonCount
  0, // hatSwitchCount
  true, // includeXAxis
  true, // includeYAxis 
  false, // includeZAxis
  false, // includeRxAxis
  false, // includeRyAxis
  false, // includeRzAxis
  false, // includeRudder
  false, // includeThrottle
  false, // includeAccelerator
  false, // includeBrake
  false  // includeSteering
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
  
  // ENCODER
  Joystick.setXAxisRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
  
  pinMode(ENCODER_OUT_A_PIN, INPUT);
  pinMode(ENCODER_OUT_B_PIN, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_OUT_A_PIN),
    encoderOutAChange,
    CHANGE);
  attachInterrupt(
    digitalPinToInterrupt(ENCODER_OUT_B_PIN),
    encoderOutBChange,
    CHANGE);
  // ENCODER

  // BTS7960 MOTOR DRIVER
  pinMode(BTS7960_RPWM_PIN, OUTPUT);
  pinMode(BTS7960_LPWM_PIN, OUTPUT);
  pinMode(BTS7960_R_EN_PIN, OUTPUT);
  pinMode(BTS7960_L_EN_PIN, OUTPUT);
  // BTS7960 MOTOR DRIVER

  // FFB gains
  mygains[0].totalGain = 100;//0-100
  mygains[0].springGain = 100;//0-100
  Joystick.setGains(mygains);
  myeffectparams[0].springMaxPosition = 1023;
  Joystick.setEffectParams(myeffectparams);
  Joystick.begin(AUTO_SEND_STATE);
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
  }
  Joystick.setXAxis(encoder_state);
  Joystick.setYAxis(0);
  // ENCODER & STEERING AXIS
  

  #if !AUTO_SEND_STATE
    Joystick.sendState();
  #endif

  // FORCE FEEDBACK

  myeffectparams[0].springMaxPosition = 1023;
  myeffectparams[0].springPosition = map(encoder_state,
      ENCODER_MIN_VALUE, ENCODER_MAX_VALUE,
      0, 1023
    );


  Joystick.setEffectParams(myeffectparams);
  Joystick.getForce(forces);
  if(forces[0] > 0){
    // Forward force
    digitalWrite(BTS7960_L_EN_PIN, HIGH);
    digitalWrite(BTS7960_R_EN_PIN, HIGH);
    analogWrite(BTS7960_RPWM_PIN, abs(forces[0]));
  }else{
    // Reverse force
    digitalWrite(BTS7960_R_EN_PIN, HIGH);
    digitalWrite(BTS7960_L_EN_PIN, HIGH);
    analogWrite(BTS7960_LPWM_PIN, abs(forces[0]));
  }
  // FORCE FEEDBACK

  #if DEBUG
    Serial.print("encoder: ");
    Serial.print(encoder_state);
    Serial.print(" forces[0]: ");
    Serial.print(forces[0]);
    Serial.print(" forces[1]: ");
    Serial.print(forces[1]);
    Serial.println();
  #endif
  delay(1);
}
