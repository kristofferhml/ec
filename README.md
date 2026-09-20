# EC Sensor ROS 2 Package

A ROS 2 package for reading Electrical Conductivity (EC) measurements from Atlas Scientific I2C sensors and publishing them as ROS topics.

## Overview

This package provides a ROS 2 node that interfaces with Atlas Scientific I2C sensors (such as EC, pH, ORP, or DO sensors) via the I2C bus. It reads sensor data at configurable intervals, applies a moving average filter, and publishes the processed values as `std_msgs/Float32` messages.

## Features

- I2C communication with Atlas Scientific sensors
- Configurable sensor address and I2C bus
- Moving average filter for measurement smoothing
- Configurable publish interval
- ROS 2 node with standard logging

## Requirements

- ROS 2 (tested with Foxy and later)
- Python 3.6+
- Raspberry Pi or other Linux system with I2C support
- Atlas Scientific I2C sensor (EC, pH, etc.)

### Hardware Requirements

- Raspberry Pi (or compatible SBC) with I2C enabled
- Atlas Scientific I2C sensor connected to the I2C bus
- Appropriate circuitry for sensor power and signal conditioning

## Installation

### From Source

```bash
# Clone the repository (or navigate to the package directory)
cd ~/ros2_ws/src

# Build the package
colcon build --packages-select ec

# Source the workspace
source install/setup.bash
```

### Enable I2C on Raspberry Pi

```bash
# Add user to i2c group
sudo usermod -aG i2c $USER

# Enable I2C kernel module
sudo raspi-config
# Navigate to: Interface Options -> I2C -> Enable

# Install I2C tools (optional, for debugging)
sudo apt-get install i2c-tools

# Reboot for changes to take effect
sudo reboot
```

## Configuration

The node is configured via environment variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `I2C_ADDRESS` | 100 | I2C address of the sensor |
| `METRIC_INTERVAL` | 10 | Publish interval in seconds |
| `READ_CMD` | R | Command to send to the sensor for reading |
| `NODE_NAME` | metric | Name of the ROS 2 node |
| `WINDOW_SIZE` | 5 | Number of measurements for moving average |

### Example Configuration

```bash
# Set custom sensor address and faster publish rate
export I2C_ADDRESS=98
export METRIC_INTERVAL=5
export WINDOW_SIZE=10

# Launch the node
ros2 run ec start
```

## Usage

### Running the Node

```bash
# With default configuration
ros2 run ec start

# With custom configuration (set environment variables first)
export I2C_ADDRESS=98
export METRIC_INTERVAL=2
ros2 run ec start
```

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/metric` | `std_msgs/Float32` | Processed EC measurement (moving average) |

### Subscribed Topics

None - the node only publishes data.

## Package Structure

```
ec/
├── ec/                          # Python package
│   ├── __init__.py
│   ├── main.py                  # Main ROS 2 node implementation
│   ├── AtlasI2C.py             # Atlas Scientific I2C driver
│   └── utils.py                # Utility functions
├── resource/
│   └── ec                      # Empty resource file (ROS 2 requirement)
├── test/                       # Test files
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml                 # ROS 2 package manifest
├── setup.py                   # Python package configuration
├── setup.cfg                   # Setuptools configuration
└── README.md                   # This file
```

## API Reference

### `main.py` - MetricNode

ROS 2 node that manages the sensor interface and message publishing.

**Key Components:**
- `MetricNode` class: Main node class inheriting from `rclpy.node.Node`
- `timer_callback()`: Called periodically to read sensor and publish data
- `main()`: Entry point for the ROS 2 node

### `AtlasI2C.py` - AtlasI2C

Low-level I2C communication with Atlas Scientific sensors.

**Key Methods:**
- `__init__(address, moduletype, name, bus)`: Initialize I2C connection
- `query(command)`: Send command and read response
- `read(num_of_bytes)`: Read data from I2C bus
- `write(cmd)`: Write command to I2C bus
- `list_i2c_devices()`: Scan for available I2C devices
- `set_i2c_address(addr)`: Set the I2C slave address

### `utils.py`

Utility functions for device management.

**Key Functions:**
- `get_device(address)`: Initialize and return an AtlasI2C device instance

## Troubleshooting

### No Device Found

If you see the message "Unable to start ec. No device found":

1. Verify I2C is enabled on your system:
   ```bash
   ls /dev/i2c-*
   ```

2. Check if your sensor is detected:
   ```bash
   sudo i2cdetect -y 1
   ```

3. Verify the `I2C_ADDRESS` environment variable matches your sensor's address.

### No Response from Probe

If the node logs "No response from probe":

1. Check the sensor is powered correctly
2. Verify the I2C connections (SDA, SCL, GND)
3. Try a different `READ_CMD` value (some sensors use different commands)

### Permission Issues

If you get permission errors accessing `/dev/i2c-*`:

```bash
# Add your user to the i2c group
sudo usermod -aG i2c $USER
# Log out and back in, or reboot
```

## Debugging

### View I2C Devices

```bash
sudo apt-get install i2c-tools
sudo i2cdetect -y 1
```

### Manual Sensor Query

```python
from ec.AtlasI2C import AtlasI2C
sensor = AtlasI2C(address=98, bus=1)
print(sensor.query("R"))  # Read measurement
print(sensor.query("i"))  # Get device info
```

## License

This package is licensed under TODO: License declaration (see package.xml).

## Maintainer

- Kristoffer Humle - [kristofferhumle@gmail.com](mailto:kristofferhumle@gmail.com)

## Contributing

Contributions are welcome. Please follow standard ROS 2 and Python coding guidelines.

### Quality Checks

This package includes quality check tests for:
- Copyright headers
- PEP 8 compliance (flake8)
- PEP 257 compliance (docstring formatting)

Run tests with:
```bash
colcon test --packages-select ec
```