// Arduino Joystick library from here: https://github.com/MHeironimus/ArduinoJoystickLibrary
#include <Joystick.h>

#define ENCODER_MAX_VALUE 2100
#define ENCODER_MIN_VALUE -2100
#define ENCODER_OUT_A_PIN 2
#define ENCODER_OUT_B_PIN 3

#define AUTO_SEND_STATE false
#define DEBUG false

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


void encoder_out_A_change() {
  // when outA changes, outA==outB means negative direction
  encoder_state += digitalRead(ENCODER_OUT_A_PIN) == digitalRead(ENCODER_OUT_B_PIN) ? -1 : 1; 
}

void encoder_out_B_change() {
  // when outB changes, outA==outB means positive direction
  encoder_state += digitalRead(ENCODER_OUT_A_PIN) == digitalRead(ENCODER_OUT_B_PIN) ? 1 : -1;
}

void setup() {
  #if DEBUG
    Serial.begin(9600); 
  #endif
  
  Joystick.begin(AUTO_SEND_STATE);
  
  Joystick.setSteeringRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
  
  pinMode(ENCODER_OUT_A_PIN, INPUT_PULLUP); //Encoder pin
  pinMode(ENCODER_OUT_B_PIN, INPUT_PULLUP); //Encoder pin
  
  attachInterrupt(
    digitalPinToInterrupt(ENCODER_OUT_A_PIN),
    encoder_out_A_change,
    CHANGE);
  attachInterrupt(
    digitalPinToInterrupt(ENCODER_OUT_B_PIN),
    encoder_out_B_change,
    CHANGE);
}

void loop() {
  
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
  
  #if !AUTO_SEND_STATE
    Joystick.sendState();
  #endif

  delay(1);
}
