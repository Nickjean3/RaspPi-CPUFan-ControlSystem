#!/usr/bin/env python3
import time
import math
import RPi.GPIO as GPIO
from ADCDevice import *

# --- Configuration ---
FAN_PIN = 17
SETPOINT = 30.0  # target temperature in Celsius

adc = ADCDevice()

def setup():
    global adc
    # GPIO setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(FAN_PIN, GPIO.OUT)
    GPIO.output(FAN_PIN, GPIO.LOW)  # fan off at start

    # ADC setup
    if adc.detectI2C(0x48):
        adc = PCF8591()
    elif adc.detectI2C(0x4b):
        adc = ADS7830()
    else:
        print("No correct I2C address found, \n"
              "Please use command 'i2cdetect -y 1' to check the I2C address! \n"
              "Program Exit. \n")
        exit(-1)

def read_temperature():
    value = adc.analogRead(0)
    voltage = value / 255.0 * 3.3
    Rt = 10 * voltage / (3.3 - voltage)
    tempK = 1 / (1 / (273.15 + 25) + math.log(Rt / 10) / 3950.0)
    tempC = tempK - 273.15
    return value, voltage, tempC

def loop():
    while True:
        value, voltage, tempC = read_temperature()

        # Bang-bang control
        if tempC > SETPOINT:
            GPIO.output(FAN_PIN, GPIO.HIGH)  # fan on
            fan_state = "ON"
        else:
            GPIO.output(FAN_PIN, GPIO.LOW)   # fan off
            fan_state = "OFF"

        print('ADC: %d | Voltage: %.2fV | Temp: %.2f°C | Setpoint: %.1f°C | Fan: %s'
              % (value, voltage, tempC, SETPOINT, fan_state))

        time.sleep(1.0)  # 1 second is fine for temperature

def destroy():
    GPIO.output(FAN_PIN, GPIO.LOW)
    GPIO.cleanup()
    adc.close()

if __name__ == '__main__':
    print('Program is starting...')
    setup()
    try:
        loop()
    except KeyboardInterrupt:
        destroy()
        print("Ending program")
