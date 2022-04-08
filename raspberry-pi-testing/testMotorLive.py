#!/usr/bin/python3

import RPi.GPIO as GPIO
from time import sleep

control1GPIO = 13
control2GPIO = 12

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(control1GPIO, GPIO.OUT)
GPIO.setup(control2GPIO, GPIO.OUT)

pwm1 = GPIO.PWM(control1GPIO, 2000)
pwm2 = GPIO.PWM(control2GPIO, 2000)
pwm1.start(0)
pwm2.start(0)

def setPwm(value):
    if value > 100 or value < -100:
        pwm1.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(0)
    if value >= 0:
        pwm1.ChangeDutyCycle(value)
        pwm2.ChangeDutyCycle(0)
    else:
        pwm1.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(-value)


while True:
    try:
        value = float(input("give pwm value (-100 ... 100): "))
    except KeyboardInterrupt:
        print()
        break
    setPwm(value)

GPIO.cleanup()