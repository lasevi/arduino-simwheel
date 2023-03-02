/*
24bit (3x8bit) shift register read test by lasevi

I still need to lower the delays.
I want the minimum delays that bring valid read results.
All of the added delay adds lag and decreases the sample rate.

*/


#define Q_pin 8  // Shift register serial output
#define PL_pin 6 // PL, parallel load input (pin9 on the HEF4021BT)
#define CP_pin 4 // CP, clock input, (pin10 on the HEF4021BT)


#define CLOCK_PULSE_MICROSECONDS 30
#define SHIFT_REGISTER_BITS 24

uint8_t wheel_serial_bit_index = 0;

uint16_t serial_data = 0;

uint16_t buttons_confirmed_to_work = 0;
unsigned long whole_read_operation_time = 0;

void asyncReadWheelButtons(uint8_t dataPin, uint8_t clockPin, uint8_t paralLoadPin, uint8_t *bit_index){
  if(*bit_index == SHIFT_REGISTER_BITS){
    *bit_index = 0;
    serial_data = 0;
    // Close the latch, switch to serial mode
    digitalWrite(paralLoadPin, 0);
    delayMicroseconds(1000);
  }
  
  uint16_t current_bit = oneBitShiftIn(dataPin, clockPin);
  //Joystick.setButton(*bit_index, !current_bit)

  if(*bit_index >= 8){
    // Skip the first 8 bits and save the bits after that
    serial_data |= current_bit << *bit_index - 8;
    if(!current_bit){
      buttons_confirmed_to_work |= 1 << *bit_index - 8;
    }
  }
  if(*bit_index == SHIFT_REGISTER_BITS-1){
    // Next loop I need to latch new values
    // Open the latch already now
    digitalWrite(paralLoadPin, 1);
  }
}

uint8_t oneBitShiftIn(uint8_t dataPin, uint8_t clockPin){
  //delayMicroseconds(CLOCK_PULSE_MICROSECONDS);  // @lasevi, Needed to add this delay
  digitalWrite(clockPin, HIGH);
  delayMicroseconds(CLOCK_PULSE_MICROSECONDS);  // @lasevi, Needed to add this delay
  uint8_t value = digitalRead(dataPin);
  digitalWrite(clockPin, LOW);
  return value;
}


void setup() {
  pinMode(PL_pin, OUTPUT);
  pinMode(CP_pin, OUTPUT);
  pinMode(Q_pin, INPUT);

  Serial.begin(9600);
}

void loop() {
  unsigned long start_time = millis();
  if(wheel_serial_bit_index == 1){
    whole_read_operation_time = 0;
  }
 
  asyncReadWheelButtons(Q_pin, CP_pin, PL_pin, &wheel_serial_bit_index);

  unsigned long end_time = millis();
  unsigned long execution_time = end_time - start_time;
  whole_read_operation_time += execution_time;
  if(wheel_serial_bit_index == SHIFT_REGISTER_BITS-1){
    Serial.println();
    Serial.print("time:");
    Serial.print(execution_time);
    Serial.print(" wholetime:");
    Serial.print(whole_read_operation_time);
    Serial.print(" data:");
    Serial.print(serial_data, BIN);
    Serial.print(" confirmed:");
    Serial.print(buttons_confirmed_to_work, BIN);
    if(buttons_confirmed_to_work == 0b111111111111111){
      Serial.print(", all!");
    }
    delay(0);
  }
  wheel_serial_bit_index++;
  //Serial.print(execution_time);
  delay(1);
}



uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
  // Arduino Default shiftIn function, edited by lasevi
  uint8_t value = 0;
  uint8_t i;
  for (i = 0; i < 8; ++i) {
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(CLOCK_PULSE_MICROSECONDS);  // @lasevi, Needed to add this delay
    if (bitOrder == LSBFIRST)
      value |= digitalRead(dataPin) << i;
    else
      value |= digitalRead(dataPin) << (7 - i);
    digitalWrite(clockPin, LOW);
  }
  return value;
}
