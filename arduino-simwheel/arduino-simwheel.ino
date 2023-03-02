// Arduino Joystick with FFB library from here:
//   https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary
#include <Joystick.h>


#define DEBUG false
#define AUTO_SEND_STATE false

#define ENCODER_OUT_A_PIN 2
#define ENCODER_OUT_B_PIN 3
#define ENCODER_MAX_VALUE 2100
#define ENCODER_MIN_VALUE -2100
#define ENCODER_RANGE ENCODER_MAX_VALUE - ENCODER_MIN_VALUE

#define BTS7960_RPWM_PIN 9
#define BTS7960_LPWM_PIN 10

#define THROTTLE_POTENTIOMETER_PIN A0
#define BRAKE_POTENTIOMETER_PIN A1

#define SHIFT_REG_DATA_PIN 8  // Shift register serial output
#define SHIFT_REG_PL_PIN 6 // PL, parallel load input (pin9 on the HEF4021BT)
#define SHIFT_REG_CP_PIN 4 // CP, clock input, (pin10 on the HEF4021BT)
// Test:
//   Parallel Load works with 400us
//   Works with 350us
//   Doesn't work with 300us
//   Too low values result in corrupted values.
#define SHIFT_REG_PL_WAIT_MICROSECONDS 400
#define SHIFT_REGISTER_BITS 24



// Both X and Y axis need to be active for FFB to work, I recall.
Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID, // hidReportId
  JOYSTICK_TYPE_JOYSTICK, // joystickType
  16, // buttonCount
  0, // hatSwitchCount
  true, // includeXAxis
  true, // includeYAxis 
  false, // includeZAxis
  false, // includeRxAxis
  false, // includeRyAxis
  false, // includeRzAxis
  false, // includeRudder
  true,  // includeThrottle
  false, // includeAccelerator
  true,  // includeBrake
  false  // includeSteering
); 

//------------------------------------------
//Note: not every pin is usable for interrupts!
//You need to connect the signal wires to pins which are usable for interrupts: Micro, Leonardo, other 32u4-based: 0, 1, 2, 3, 7
volatile int32_t encoder_state = 0; // volatile is using RAM this is because we will use interrupts

// FFB
Gains mygains[2];
EffectParams myeffectparams[2];
int32_t forces[2] = {0};

// Wheel shift registers
uint8_t wheel_serial_bit_index = 0;
#if DEBUG
  uint16_t buttons_serial_data = 0;
  uint16_t buttons_confirmed_to_work = 0;
#endif

uint8_t count = 0;



void setPwmFrequency(){
  // Set-up hardware PWM on the Arduino UNO/Pro Micro on digital pins D9 and D10
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);  // Enable PWM outputs for OC1A and OC1B on digital pins 9, 10
  //TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);   // Set fast PWM and prescaler of 8 on timer 1
  //ICR1 = 999;                                     // Set the PWM frequency to 2kHz (16MHz / (8 * (999 + 1)))
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);     // Set fast PWM and prescaler of 1 on timer 1
  ICR1 = 800;                                       // Set the PWM frequency to 20kHz (16MHz / (1 * (799 + 1)))
  OCR1A = 0;                                        // Set duty-cycle to 0% on D9
  OCR1B = 0.5*ICR1;                                 // Set duty-cycle to 50% on D10
}


void setPwmDutyCycle(int pin, int duty_cycle){
  /*
    I got some strange behaviour with analogWrite and ICR1 799.
    It's like duty cycle rises too slowly first and jumps to 100% at the end.
    I suspect analogWrite only works on when ICR1 is the default value (255?).
   */
  if(pin == 9){
    OCR1A = duty_cycle;
  }
  if(pin == 10){
    OCR1B = duty_cycle;
  }
}


void asyncReadWheelButtons(){
  /*
  * This reads one button per function call, so it
  *   takes SHIFT_REGISTER_BITS times to read all buttons.
  *
  * I made this to reduce read latency from ~14ms to <1ms,
  *   compared to reading all values at once.
  */
  if(wheel_serial_bit_index == SHIFT_REGISTER_BITS){
    wheel_serial_bit_index = 0;
    // Close the latch, switch to serial mode
    digitalWrite(SHIFT_REG_PL_PIN, LOW);
    delayMicroseconds(SHIFT_REG_PL_WAIT_MICROSECONDS);
  }
  
  // Pulse the clock and read a bit
  digitalWrite(SHIFT_REG_CP_PIN, HIGH);
  uint16_t current_bit = digitalRead(SHIFT_REG_DATA_PIN);
  digitalWrite(SHIFT_REG_CP_PIN, LOW);

  if(wheel_serial_bit_index < 8){
    // Skip the first 8 bits, they are not useful.
    // The first bits just statically output something, I guess
    // it's the steering wheel ID or revision or something like that.
    wheel_serial_bit_index++;
    return;
  }

  // And save the bits after the 8th bit
  uint8_t button_index = wheel_serial_bit_index - 8;

  // Set the buttons, they are pressed when low
  Joystick.setButton(button_index, !current_bit);

  #if DEBUG
    buttons_serial_data |= current_bit << button_index;
    if(!current_bit){
      buttons_confirmed_to_work |= 1 << button_index;
    }
  #endif

  if(wheel_serial_bit_index == SHIFT_REGISTER_BITS-1){
    // Next loop I need to latch new values
    // Opening the latch at this time saves a tiny amount of time #premature-optimization
    digitalWrite(SHIFT_REG_PL_PIN, HIGH);
  }
  wheel_serial_bit_index++;
}


void encoderSaturation() {
  if(encoder_state > ENCODER_MAX_VALUE){
    encoder_state = ENCODER_MAX_VALUE;
  }
  else if(encoder_state < ENCODER_MIN_VALUE){
    encoder_state = ENCODER_MIN_VALUE;
  }
}


void encoderOutAChange() {
  // when outA changes, outA==outB means negative direction
  encoder_state += digitalRead(ENCODER_OUT_A_PIN) == digitalRead(ENCODER_OUT_B_PIN) ? -1 : 1;
  encoderSaturation();
}


void encoderOutBChange() {
  // when outB changes, outA==outB means positive direction
  encoder_state += digitalRead(ENCODER_OUT_A_PIN) == digitalRead(ENCODER_OUT_B_PIN) ? 1 : -1;
  encoderSaturation();
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
  // BTS7960 MOTOR DRIVER


  // FFB setup
  mygains[0].totalGain = 100;//0-100
  mygains[0].springGain = 100;//0-100
  Joystick.setGains(mygains);
  myeffectparams[0].springMaxPosition = ENCODER_RANGE;
  Joystick.setEffectParams(myeffectparams);
  // FFB setup


  // Pedals
  pinMode(THROTTLE_POTENTIOMETER_PIN, INPUT);
  pinMode(BRAKE_POTENTIOMETER_PIN, INPUT);
  Joystick.setThrottleRange(0, 1023);  // Same range as the potentiometer
  Joystick.setBrakeRange(0, 1023);
  // Pedals


  // BUTTONS
  pinMode(SHIFT_REG_DATA_PIN, INPUT);
  pinMode(SHIFT_REG_PL_PIN, OUTPUT);
  pinMode(SHIFT_REG_CP_PIN, OUTPUT);
  // BUTTONS


  // Joystick setup
  Joystick.begin(AUTO_SEND_STATE);
  // Joystick setup


  // MISC
  setPwmFrequency();
  // MISC
}


void loop() {
  // THROTTLE & BRAKE AXIS
  uint16_t throttle = analogRead(THROTTLE_POTENTIOMETER_PIN);
  uint16_t brake = analogRead(BRAKE_POTENTIOMETER_PIN);
  Joystick.setThrottle(throttle);
  Joystick.setBrake(brake);
  // THROTTLE & BRAKE AXIS


  // BUTTONS
  asyncReadWheelButtons();
  // BUTTONS


  // STEERING AXIS
  Joystick.setXAxis(encoder_state);
  myeffectparams[0].springPosition = map(encoder_state,
      ENCODER_MIN_VALUE, ENCODER_MAX_VALUE,
      0, ENCODER_RANGE
    );
  Joystick.setEffectParams(myeffectparams);
  Joystick.getForce(forces);
  // STEERING AXIS


  #if !AUTO_SEND_STATE && !DEBUG
    Joystick.sendState();
  #endif


  // FORCE FEEDBACK
  // Scale the -255...255 force to the PWM value range
  int ffbPWM = map(abs(forces[0]), 0,  255, 0, ICR1);
  if(forces[0] > 0){
    // Forward force (R)
    analogWrite(BTS7960_RPWM_PIN, ffbPWM);
    analogWrite(BTS7960_LPWM_PIN, 0);
  }else{
    // Reverse force (L)
    analogWrite(BTS7960_RPWM_PIN, 0);
    analogWrite(BTS7960_LPWM_PIN, ffbPWM);
  }
  // FORCE FEEDBACK


  #if DEBUG
    Serial.print("encoder:");
    Serial.print(encoder_state);
    Serial.print(" ffbPWM:");
    Serial.print(ffbPWM);
    Serial.print(" thr:");
    Serial.print(throttle);
    Serial.print(" brk:");
    Serial.print(brake);
    // Shift register debug
    Serial.print(" buttons:");
    Serial.print(buttons_serial_data, BIN);
    Serial.print(" confirmed:");
    Serial.print(buttons_confirmed_to_work, BIN);
    if(buttons_confirmed_to_work == 0b111111111111111){
      Serial.print(", all!");
    }
    // Shift register debug
    Serial.println();
  #endif
}
