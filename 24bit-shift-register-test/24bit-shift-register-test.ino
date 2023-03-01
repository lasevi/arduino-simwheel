/*
24bit (3x8bit) shift register read test by lasevi

I still need to lower the delays.
I want the minimum delays that bring valid read results.
All of the added delay adds lag and decreases the sample rate.

*/


#define PL_pin 14 // PL, parallel load input (pin9 on the HEF4021BT)
#define CP_pin 15 // CP, clock input, (pin10 on the HEF4021BT)
#define Q_pin 16  // Shift register serial output


uint8_t shiftregister1 = 0;
uint8_t shiftregister2 = 0;
uint8_t shiftregister3 = 0;


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
  

  // Read all the 3 8bit shift registers
  shiftregister1 = shiftIn(Q_pin, CP_pin, MSBFIRST);
  shiftregister2 = shiftIn(Q_pin, CP_pin, MSBFIRST);
  shiftregister3 = shiftIn(Q_pin, CP_pin, MSBFIRST);

  Serial.print(shiftregister1, BIN);
  Serial.print(" ");
  Serial.print(shiftregister2, BIN);
  Serial.print(" ");
  Serial.print(shiftregister3, BIN);
  Serial.print(" ");
  Serial.print(count);
  Serial.println();
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
