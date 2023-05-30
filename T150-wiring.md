# Thrustmaster T150 wiring

## Device pinouts

### Encoder
- Grey=GND
- Red=5V
- Blue&Brown=Signal

### Wheel PCB pinout
Starting from the 45deg cut:
- pin1 (green)
  - just a 100kOhm pulldown resistor?
- pin2 (blue)
  - GND
- pin3 (white)
  - goes under the chip, can't trace. But has to be serial out.
- pin4 (orange)
  - PL, parallel load input (pin9 on the HEF4021BT)
- pin5 (red)
  - CP, clock input, (pin10 on the HEF4021BT)
- pin6 (grey)
  - Vdd (Vcc) (3...15V)

The wheel PCB has 3x HEF4021BT 8 bit PISO shift registers.

### Wheelbase buttons
Starting from red wire:
- pin1 (red)
  - MODE button (no pullup resistor)
- pin2 (black)
  - SW1 (no pullup)
- pin3 (green)
  - GND
- pin4 (yellow)
  - R3_BTN (no pullup)
- pin5 (blue)
  - MODE_LED R (red led positive terminal)
  - No current limiting resistor
- pin6 (brown)
  - MODE_LED G (green led positive terminal)
  - No current limiting resistor
- pin7 (white)
  - L3_BTN (no pullup)

The led has three terminals, and the middle one is tied to GND (pin3).

The motor has a 50kOhm thermistor.

### PSU
It's a simple 24V power supply with an optocoupler controlled by a 5V input signal that turns the PSU on when pulled high.
