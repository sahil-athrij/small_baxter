from time import sleep

import Jetson.GPIO as GPIO

Motor1 = 38  # Input Pin
Motor2 = 36  # Input Pin

Motor4 = 26
Motor5 = 24

GPIO.setmode(GPIO.BOARD)
print(GPIO.gpio_pin_data.get_data()[-1])
GPIO.setup(Motor1, GPIO.OUT)
GPIO.setup(Motor2, GPIO.OUT)

GPIO.setup(Motor4, GPIO.OUT)
GPIO.setup(Motor5, GPIO.OUT)


def forward_async():
    GPIO.output(Motor1, GPIO.HIGH)
    GPIO.output(Motor2, GPIO.LOW)

    GPIO.output(Motor4, GPIO.HIGH)
    GPIO.output(Motor5, GPIO.LOW)


def reverse_async():
    GPIO.output(Motor1, GPIO.LOW)
    GPIO.output(Motor2, GPIO.HIGH)

    GPIO.output(Motor4, GPIO.LOW)
    GPIO.output(Motor5, GPIO.HIGH)


def turn_right_async():
    GPIO.output(Motor1, GPIO.HIGH)
    GPIO.output(Motor2, GPIO.LOW)


def turn_left_async():
    GPIO.output(Motor4, GPIO.HIGH)
    GPIO.output(Motor5, GPIO.LOW)


def stop():
    GPIO.output(Motor1, GPIO.LOW)
    GPIO.output(Motor2, GPIO.LOW)

    GPIO.output(Motor4, GPIO.LOW)
    GPIO.output(Motor5, GPIO.LOW)



def forward(duratrion):
    forward_async()
    sleep(duratrion)
    stop()


def reverse(duration):
    reverse_async()
    sleep(duration)
    stop()


def turn_right(duration):
    turn_right_async()
    sleep(duration)
    stop()


def turn_left(duration):
    turn_right_async()
    sleep(duration)
    stop()

