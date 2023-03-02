/*
24bit (3x8bit) shift register read test by lasevi

I still need to lower the delays.
I want the minimum delays that bring valid read results.
All of the added delay adds lag and decreases the sample rate.

*/

#define DEBUG true

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

uint8_t wheel_serial_bit_index = 0;

#if DEBUG
  uint16_t serial_data = 0;
  uint16_t buttons_confirmed_to_work = 0;
#endif

unsigned long whole_read_operation_time = 0;

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
    #if DEBUG
      serial_data = 0;
    #endif
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
    return;
  }

  // And save the bits after the 8th bit
  uint8_t button_index = wheel_serial_bit_index - 8;

  // Set the buttons, they are pressed when low
  //Joystick.setButton(button_index, !current_bit)

  #if DEBUG
    serial_data |= current_bit << button_index;
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


void setup() {
  pinMode(SHIFT_REG_PL_PIN, OUTPUT);
  pinMode(SHIFT_REG_CP_PIN, OUTPUT);
  pinMode(SHIFT_REG_DATA_PIN, INPUT);

  Serial.begin(9600);
}

void loop() {
  // Timer
  unsigned long start_time = millis();
  if(wheel_serial_bit_index == 1){
    whole_read_operation_time = 0;
  }

  asyncReadWheelButtons();

  // Timer
  unsigned long end_time = millis();
  unsigned long execution_time = end_time - start_time;
  whole_read_operation_time += execution_time;

  // Prints
  if(wheel_serial_bit_index == SHIFT_REGISTER_BITS-1){
    Serial.println();
    Serial.print("time:");
    Serial.print(execution_time);
    Serial.print(" wholetime:");
    Serial.print(whole_read_operation_time);
    #if DEBUG
      Serial.print(" data:");
      Serial.print(serial_data, BIN);
      Serial.print(" confirmed:");
      Serial.print(buttons_confirmed_to_work, BIN);
      if(buttons_confirmed_to_work == 0b111111111111111){
        Serial.print(", all!");
      }
    #endif
  }

  wheel_serial_bit_index++;
  delay(5);
}
