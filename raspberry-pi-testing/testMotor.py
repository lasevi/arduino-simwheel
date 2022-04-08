#!/usr/bin/python3

import RPi.GPIO as GPIO
from time import sleep

control1GPIO = 13
control2GPIO = 12

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(control1GPIO, GPIO.OUT)
GPIO.setup(control2GPIO, GPIO.OUT)

pwm1 = GPIO.PWM(control1GPIO, 200)
pwm2 = GPIO.PWM(control2GPIO, 200)
pwm1.start(0)
pwm2.start(0)

for duty in range(0, 101, 1):
    pwm1.ChangeDutyCycle(duty)
    sleep(0.01)
sleep(0.5)
for duty in range(100, -1, -1):
    pwm1.ChangeDutyCycle(duty)
    sleep(0.01)
sleep(0.5)
for duty in range(0, 101, 1):
    pwm2.ChangeDutyCycle(duty)
    sleep(0.01)
sleep(0.5)
for duty in range(100, -1, -1):
    pwm2.ChangeDutyCycle(duty)
    sleep(0.01)
sleep(0.5)

GPIO.setup(control1GPIO, GPIO.IN)
GPIO.setup(control2GPIO, GPIO.IN)
GPIO.cleanup()