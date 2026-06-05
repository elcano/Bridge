# Stage 2 Graphic Simulator

A Python-based 2D vehicle simulator for the Elcano autonomous vehicle project.

The simulator models:

* delayed throttle-to-speed behavior
* momentum/friction
* braking
* steering and heading updates
* vehicle position tracking
* CSV logging for analysis

## Requirements

* Python 3.12+
* pygame

Install pygame:
pip install pygame

## Run

python stage2\_simulator.py

## Controls

Up Arrow   = throttle
Down Arrow = brake
Left/Right = steer
R          = reset

## Output

The simulator creates a timestamped CSV log containing:

* time
* X/Y position
* heading
* speed
* steering angle
* throttle
* brake state

## Screenshots

See the screenshots folder for:

* simulator output
* CSV/logging examples

\## Live Router Telemetry Simulator



File:

stage2\_live\_router\_simulator.py



This version connects directly to the Router Arduino Due over USB serial and displays live vehicle position from Router telemetry packets.



\### Additional Requirements



Install:



pip install pygame pyserial



\### Router Setup



1. Upload the Stage 1 + Stage 3 Router code to the Arduino Due.
2. Use the Arduino Due Programming Port.
3. Set the baud rate to 115200.
4. Close Arduino Serial Monitor before running the simulator.



\### Run



python stage2\_live\_router\_simulator.py



\### Serial Configuration



Update the serial port if necessary:



```python

PORT = "COM12"

BAUD = 115200

