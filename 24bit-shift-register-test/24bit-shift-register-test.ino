/*
24bit (3x8bit) shift register read test by lasevi

I still need to lower the delays.
I want the minimum delays that bring valid read results.
All of the added delay adds lag and decreases the sample rate.

*/


#define Q_pin 8  // Shift register serial output
#define PL_pin 6 // PL, parallel load input (pin9 on the HEF4021BT)
#define CP_pin 4 // CP, clock input, (pin10 on the HEF4021BT)

uint8_t count = 0;

void setup() {
  pinMode(PL_pin, OUTPUT);
  pinMode(CP_pin, OUTPUT);
  pinMode(Q_pin, INPUT);

  Serial.begin(9600);
}

void loop() {
  // Load the parallel values to the registers
  digitalWrite(PL_pin,1);
  delay(20);
  digitalWrite(PL_pin,0);
  delay(20);
  
  uint8_t pressed_count = 0;
  // Read all the 3 8bit shift registers
  uint32_t shiftregisters = shiftIn(Q_pin, CP_pin, MSBFIRST) <<16;
  shiftregisters |= shiftIn(Q_pin, CP_pin, MSBFIRST) <<8;
  shiftregisters |= shiftIn(Q_pin, CP_pin, MSBFIRST);

  for(int i=0;i<16;i++){
    uint32_t selector_bit = 1 << i;
    uint32_t selected_bit = shiftregisters & selector_bit;
    if(selected_bit > 0){
      Serial.print(0);
    }else{
      Serial.print(1);
      pressed_count++;
    }
  }
  Serial.print(" ");
  Serial.print(pressed_count);
  Serial.println();
  count++;
  delay(50);
}



uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
  // Arduino Default shiftIn function, edited by lasevi
  uint8_t value = 0;
  uint8_t i;
  for (i = 0; i < 8; ++i) {
    delay(1);  // @lasevi, Needed to add these delays
    digitalWrite(clockPin, HIGH);
    delay(1);  // @lasevi, Needed to add these delays
    if (bitOrder == LSBFIRST)
      value |= digitalRead(dataPin) << i;
    else
      value |= digitalRead(dataPin) << (7 - i);
    digitalWrite(clockPin, LOW);
  }
  return value;
}
