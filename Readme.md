# Motor Control System

A Python-based CAN motor control system featuring a real-time GUI interface for precise position control and monitoring.
![2025-01-23_21-44](https://github.com/user-attachments/assets/d096390e-c245-4a2b-87aa-8b5a66d022e7)


## Overview

This project provides a comprehensive solution for controlling and monitoring CAN-based motors with features including:
- Real-time position and velocity monitoring
- Interactive GUI control panel
- Dynamic motion parameter adjustment
- Real-time data visualization with auto-scaling charts
- Comprehensive logging system

## Prerequisites

- Python 3.8 or higher
- CAN interface hardware
- Linux operating system (for CAN support)

## Installation

1. Clone the repository:

```bash
git clone https://github.com/yourusername/motor-control-system.git
cd motor-control-system
```
2. Install required packages:

```bash
pip install -r requirements.txt
```

## Project Structure

- `MultiMotorControl_PP.py`: Core motor control implementation
- `motor_control_gui.py`: GUI implementation
- `requirements.txt`: Python package dependencies
- `LICENSE`: MIT license file

## Usage
1. Configure CAN interface:

```bash

sudo ip link set can0 type can bitrate 1000000
python eRobControl_PP.py
python MultiMotorControl_PP.py
```



## Features

### Motor Control
- Position control with angle input
- Velocity, acceleration, and deceleration parameter adjustment
- Real-time position and velocity monitoring

### GUI Interface
- Real-time position and velocity plots
- Auto-scaling axes for optimal data visualization
- Operation logging panel
- Parameter input validation

### Data Visualization
- Dynamic chart scaling
- 10-second time window display
- Dual-axis display for position and velocity
- Anti-aliased rendering

## Configuration

Default motor parameters:
- Encoder resolution: 524288 pulses/revolution
- Default velocity: 5°/s
- Default acceleration: 5°/s²
- Default deceleration: 5°/s²

## Contributing

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a new Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Authors

[Your Name] - Initial work

## Acknowledgments

- PyQt5 team for the GUI framework
- python-can developers for CAN support
- CANopen community for protocol implementation

