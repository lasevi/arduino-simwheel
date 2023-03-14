# Arduino simwheel

I had a Thrustmaster T150 wheel with a fried motherboard.

I thought: I had the electronics already, so why wouldn't I resurrect my old wheel and have some fun?

To do this yourself, you need:
- An Arduino Leonardo/Pro Micro (or something with an Atmega32u4 processor)
- A simulator wheel with a force feedback motor & encoder.
- A motor driver (I used a BTS7960)
- Patience

The correct Arduino code is in `arduino-simwheel` directory.

This was a really fun challenge.

<br>
<br>
<br>
 
 ## Device pinouts
 
 ### Encoder
 - Grey=GND
 - Red=5V
 - Blue&Brown=Signal
 
### Wheel PCB pinout, start from the 45deg cut:
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

The wheel PCB has 3x HEF4021BT 8 bit PISO shift registers.

### Wheelbase buttons, starting from red wire
- pin1
  - MODE button (no pullup resistor)
  - red wire
- pin2
  - SW1 kytkin (no pullup)
  - black wire
- pin3 
  - GND?
  - green wire
- pin4
  - R3_BTN (no pullup)
  - yellow wire
- pin5
  - MODE_LED R (red led positive terminal)
  - No current limiting resistor
  - blue wire
- pin6
  - MODE_LED G (green led positive terminal)
  - No current limiting resistor
  - brown wire
- pin7
  - L3_BTN (no pullup)
  - white wire

The led has three terminals, and the middle one is tied to GND (pin3).

The motor has a 50kOhm thermistor.

### PSU
It's a simple (supposedly 24V) power supply.

- Optocoupler controlled by a 5V input signal that turns the PSU on when pulled high.
- The octocoupler drives a simple triac.
- It has a simple transformer.
- Then it has a simple full bridge rectifier built with separate diodes.
- And a 2220uF 35V filter capasitor on the output.

If it's the same/similar PSU that the T300rs or T500rs have it's prone to overheating:
- The PSU's internal resistance goes up which decreases output power and increases heat even more
- The optocoupler dies when exposed to heat => the whole PSU stops working

=> I need appropriate cooling for this part.
