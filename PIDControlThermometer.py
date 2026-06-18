#!/usr/bin/env python3
import time
import math
import RPi.GPIO as GPIO
import csv
import os
from datetime import datetime
from ADCDevice import *

# --- Configuration ---
FAN_PIN = 17

SETPOINT = 36.0 # TARGET TEMP

Kp = 10.0   # proportional gain
Ki = 1.2    # integral gain
Kd = 2.0    # derivative gain
FAN_MIN = 30  # minimum duty cycle to spin the fan

# Derivative filter coefficient (0 = no filter, 0.9 = heavy filter)
DERIV_FILTER = 0.7

adc = ADCDevice()
pwm = None

def setup():
    global adc, pwm
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(FAN_PIN, GPIO.OUT)
    pwm = GPIO.PWM(FAN_PIN, 100)
    pwm.start(0)
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
    # create a timestamped filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"fan_log_{timestamp}.csv"
    
    # --- Initialization ---
    # Take a real first reading so prev_error starts at the actual error
    # instead of 0, which would cause a huge derivative spike on the first tick
    _, _, tempC = read_temperature()
    integral = 0.0
    prev_error = tempC - SETPOINT
    filtered_deriv = 0.0
    prev_time = time.time()
    
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['time_s', 'temp_c', 'error', 'p', 'i', 'd', 'output'])

        start_time = time.time()

        while True:
            # --- Sensor Reading ---
            value, voltage, tempC = read_temperature()

            # Calculate time elapsed since last loop iteration
            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time

            # Guard against zero or negative dt on the first tick
            if dt <= 0:
                dt = 1.0

            # Error = how far we are from the target temperature
            # Positive means too hot, negative means too cold
            error = tempC - SETPOINT

            # --- Integral Term ---
            # Only accumulate when error is positive or integral is still
            # unwinding, preventing windup when we're below setpoint
            if error > 0 or integral > 0:
                integral += error * dt

            # Clamp integral to prevent excessive windup
            # Max contribution to output = Ki * 20
            integral = max(0, min(20, integral))

            # --- Derivative Term ---
            # Raw derivative = rate of change of error since last tick
            # Differentiating noisy ADC readings amplifies that noise,
            # so we apply a low-pass filter to smooth it out
            raw_deriv = (error - prev_error) / dt
            filtered_deriv = DERIV_FILTER * filtered_deriv + (1 - DERIV_FILTER) * raw_deriv
            prev_error = error

            # --- PID Output ---
            p_term = Kp * error
            i_term = Ki * integral
            d_term = Kd * filtered_deriv

            # Sum the three terms, offset by FAN_MIN so the fan
            # keeps spinning even when the controller output is near zero
            output = FAN_MIN + p_term + i_term + d_term

            # Clamp to valid PWM range
            output = max(0, min(100, output))
            pwm.ChangeDutyCycle(output)

            # --- Logging ---
            elapsed = current_time - start_time

            writer.writerow([f"{elapsed:.2f}", f"{tempC:.2f}", f"{error:.2f}",
                             f"{p_term:.2f}", f"{i_term:.2f}", f"{d_term:.2f}",
                             f"{output:.2f}"])

            # Flush every row so data isn't lost if the program is interrupted
            csvfile.flush()

            print('Temp: %.2f°C | Error: %.2f | P: %.1f | I: %.1f | D: %.1f | Output: %.1f%%'
                  % (tempC, error, p_term, i_term, d_term, output))

            time.sleep(1.0)
        
def destroy():
    pwm.stop()
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
