 # raspi-BTS7960
 
 ## T150
 Enkooderin pinout:
 - Harmaa=GND
 - Punanen=5V
 - Sininen&ruskia=signaali
 
Wheel PCB pinout, start from the 45deg cut:
- pin1
  - just a 100kOhm pulldown resistor?
  - green wire
- pin2
  - GND
  - blue wire
- pin3
  - goes under the chip, can't trace. But has to be serial out.
  - white wire
- pin4
  - PL, parallel load input (pin9 on the HEF4021BT)
  - orange wire
- pin5
  - CP, clock input, (pin10 on the HEF4021BT)
  - red wire
- pin6
  - Vdd (Vcc) (3...15V)
  - grey wire
