#!/usr/bin/python3

import RPi.GPIO as GPIO

encoderCLKpin = 2
encoderDTpin = 3
encoderCounter = 0

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(encoderCLKpin, GPIO.IN)
GPIO.setup(encoderDTpin, GPIO.IN)

lastEncoderCLKstate = GPIO.input(encoderCLKpin)
lastEncoderDTstate = GPIO.input(encoderDTpin)

loopCounter = 0
try:
    while True:
        encoderCLKstate = GPIO.input(encoderCLKpin)
        encoderDTstate = GPIO.input(encoderDTpin)
        if encoderCLKstate and not lastEncoderCLKstate:
            # clock rising edge
            if encoderDTstate:
                encoderCounter += 1
            else:
                encoderCounter -= 1
        # double data rate
        elif lastEncoderCLKstate and not encoderCLKstate:
            # clock falling edge
            if not encoderDTstate:
                encoderCounter += 1
            else:
                encoderCounter -= 1
        lastEncoderCLKstate = encoderCLKstate
        lastEncoderDTstate = encoderDTstate
        if encoderCounter < 0:
            encoderCounter = 0
        loopCounter += 1
        if loopCounter % 1000 == 0:
            print("en1:{},en2:{},counter:{}.".format(encoderCLKstate, encoderDTstate, encoderCounter))
finally:
    GPIO.cleanup()
