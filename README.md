# RaspPi CPU Fan Control System

A Raspberry Pi project that uses a PID controller to regulate CPU temperature by dynamically adjusting a cooling fan's speed via PWM.

## How It Works

The system reads temperature from a thermistor connected through an I2C ADC module (PCF8591 or ADS7830), computes a PID control output, and drives a fan using hardware PWM. The fan speed is updated every second based on how far the current temperature is from the target setpoint.

## PID Controller

The core of the project is a PID (Proportional-Integral-Derivative) controller, which combines three terms to produce a smooth and accurate fan speed output:

**Proportional (P)** reacts to the current error — how far the temperature is from the setpoint right now. A large error means a large correction. On its own, P can get close to the target but typically leaves a small steady-state offset.

**Integral (I)** accumulates error over time. If the temperature sits above the setpoint for an extended period, the integral term builds up and pushes the fan harder to eliminate the remaining offset that P alone can't close.

**Derivative (D)** reacts to the rate of change of the error. If the temperature is rising quickly, D increases fan speed preemptively before the error gets large. A low-pass filter is applied to the derivative term to reduce noise amplification from the ADC.

## Features

- Auto-detects PCF8591 or ADS7830 ADC over I2C
- Configurable setpoint, Kp, Ki, Kd gains at the top of the script
- Integral windup protection with adjustable clamp
- Derivative spike prevention on startup
- CSV logging with timestamped filenames for graphing and analysis

## Hardware

- Raspberry Pi
- 5V CPU Fan (Freenove Kit)
- NTC thermistor (10kΩ, B=3950)
- PCF8591 or ADS7830 ADC module
- 10kΩ, 1kΩ resistors
- S8050 NPN Transistor
- Rectifier Diode (Flyback)
- Jumper wires

## Files

| File | Description |
|------|-------------|
| `PIDControlThermometer.py` | Main PID fan controller |
| `ControlThermometer.py` | Basic on/off control |
| `ADCDevice.py` | ADC driver library |
