# Stage 2 Graphic Simulator

A Python-based 2D vehicle simulator for the Elcano autonomous vehicle project.

The simulator models:
- delayed throttle-to-speed behavior
- momentum/friction
- braking
- steering and heading updates
- vehicle position tracking
- CSV logging for analysis

## Requirements
- Python 3.12+
- pygame

Install pygame:
pip install pygame

## Run
python stage2_simulator.py

## Controls
Up Arrow   = throttle
Down Arrow = brake
Left/Right = steer
R          = reset

## Output
The simulator creates a timestamped CSV log containing:
- time
- X/Y position
- heading
- speed
- steering angle
- throttle
- brake state

## Screenshots
See the screenshots folder for:
- simulator output
- CSV/logging examples