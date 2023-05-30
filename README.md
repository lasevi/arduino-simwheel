# Arduino Simwheel
This repository contains the necessary code and information to resurrect a dead Thrustmaster T150 wheel using an Arduino and a motor driver.

## Requirements
To replicate this project, you will need the following components:
- Arduino Leonardo/Pro Micro (or a similar device with an Atmega32u4 processor)
- T150 or any other simulator wheel with a force feedback motor and encoder
- Motor driver (BTS7960 was used in this project)
- Patience

## Contents
- The Arduino code required for the simwheel can be found in the `arduino-simwheel` directory.
- For wiring and pinout information specific to the T150 wheel, please refer to `T150-wiring.md`.
- To test the Thrustmaster wheel buttons, you can use the code in `24bit-shift-register-test` folder.

## Installation and Usage
### 1. Verify that all components are working correctly

#### Test your encoder
You may want to first test your encoder to see how many steps it has, and edit the code accordingly.

#### Test your wheel buttons
Also, you may want to test the wheel buttons (shift registers) to see if they work. You can see the test code for the wheel buttons in `24bit-shift-register-test` folder.
 - If the wheel buttons don't behave as they should, you can add a big capacitor to the 5V line - the shift registers seem to draw a very short but high current pulse.
 - If the shift registers still give bad output you can increase the SHIFT_REG_PL_WAIT_MICROSECONDS, it may help.

### 2. Do the necessary edits to the code and flash it

### 3. Have fun!

## Contributing

If you encounter any issues or would like to contribute in any way, please feel free to create an issue or reach out to me. Your input is valuable and helps improve the project.

## License
This project is licensed under the [MIT License](LICENSE). You can find a copy of the license file in the repository.
