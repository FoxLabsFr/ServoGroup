# ServoGroup Library

ServoGroup is an Arduino library for controlling multiple servos simultaneously using linear interpolation. You can send an array of positions and smoothly drive all the servos in the group to their targets within a specified timing.

It currently supports two types of servo groups:

- **I2C-based servos** using a PCA9685 board
- **Standard PWM servos** connected directly to I/O pins

## 🚀 Features

- [x] **Multiple servo control** via I2C PWM drivers (PCA9685)
- [x] **Smooth motion interpolation** with configurable duration
- [x] **Position tracking** and state management
- [x] **Individual servo configuration** with min/max limits, offsets, and inversion
- [x] **Direct JSON status output** for easy integration with web interfaces
- [x] **Support Standard PWM servo**
- [ ] **Support mixed groups (I2C driven servo and classic PWM servo)**
- [ ] **Support multiple servo groups in one project with IDs**

## 🏗️ Hardware & Specifications

**Required Hardware:**

- **PCA9685 PWM Servo Driver** - (For I2C mode).
- **Arduino-compatible board** - ESP32, Arduino Uno, etc.

## 🛠️ Getting Started

### PlatformIO

Add to your `platformio.ini`:

```ini
lib_deps =
  https://github.com/FoxLabsFr/ServoGroup.git
```

## 🚀 Usage

### Angle Format

**Important:** To avoid floating-point calculations, angles are specified as `degree*10`. For example, 90° should be passed as `900`, and 45.5° should be passed as `455`. This applies to all methods that accept angle parameters.

### 💡 Basic Examples

```cpp
#include <ServoGroup.h>

ServoGroup servoGroup1;

void setup() {
  // I2C Mode (using PCA9685)
  servoGroup1.setIds(0x40, {0, 1, 2, 3, 4});
  servoGroup1.setDefaultPosition({0, 1750, 1373, 950, 900});

  // Direct PWM Mode (using GPIO pins)
  // For ESP8266: use D0, D1, D2, D3, D4
  // servoGroup1.setIds("pwm", {D0, D1, D2, D3, D4});

  // For Arduino: use 2, 3, 4, 5, 6
  // servoGroup1.setIds("pwm", {2, 3, 4, 5, 6});

  // Optional config
  // servoGroup1.setMinPulse({150, 150, 150, 150, 150});
  // servoGroup1.setMaxPulse({600, 600, 600, 600, 600});
  // servoGroup1.setOffsets({-137, 74, -117, -169, -6});
  // servoGroup1.setInverts({-1, 1, 1, -1, 1});
  // servoGroup1.setMinAngles({200, 200, 200, 200, 200});  // 20° minimum
  // servoGroup1.setMaxAngles({1600, 1600, 1600, 1600, 1600});  // 160° maximum

  // Init servo at default position
  servoGroup1.init();

  // Optional: Enable debug logging
  // servoGroup1.init(Serial);
}

void loop() {
  servoGroup1.update();
}
```

### 🔌 Hardware Modes

**I2C Mode (PCA9685):**

- Use `setIds(i2c_address, {servo_ids...})`
- Requires PCA9685 PWM driver board
- Supports up to 16 servos per driver

**Direct PWM Mode (GPIO):**

- Use `setIds("pwm", {pin_numbers...})`
- Uses Arduino Servo library
- Direct control via GPIO pins
- Limited by available PWM pins on your board

### ⚡ Real-Time Updates

The update method must be called in real-time to continuously update motor positions. Avoid using blocking instructions like delay in your code, as they will freeze the entire logic and prevent update from being called as required.

### ⏱️ Non-Blocking Waiting

To handle non-blocking waiting, you can use the millis() function to track elapsed time without freezing the program. Here's an example:

```cpp
unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 second

void loop() {
  unsigned long currentMillis = millis();

  // Check if the interval has elapsed
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Perform your periodic task here
    Serial.println("1 second has passed");
  }

  // Always call update in the loop
  servoGroup1.update();
}
```

This approach ensures that your program remains responsive while performing periodic tasks.

## 📚 API Reference

### 🔧 Constructor

```cpp
ServoGroup()
```

### 🔧 Configuration Options

#### JSON Interface (Optional)

The library includes JSON interface methods that are enabled by default but can be disabled to save RAM:

```cpp
#define SERVOGROUP_DISABLE_JSON
```

### ⚙️ Initialization Methods

| Method                 | Parameters                | Description                                  |
| ---------------------- | ------------------------- | -------------------------------------------- |
| `setIds()`             | `i2c_address`, `{ids...}` | Set I2C address and servo IDs (I2C mode)     |
| `setIds()`             | `"pwm"`, `{pins...}`      | Set GPIO pins for direct PWM control         |
| `setDefaultPosition()` | `{positions...}`          | Set default positions for all servos         |
| `setMinPulse()`        | `{minPulses...}`          | Set minimum pulse widths (μs) for all servos |
| `setMaxPulse()`        | `{maxPulses...}`          | Set maximum pulse widths (μs) for all servos |
| `setMinAngles()`       | `{minAngles...}`          | Set soft minimum angle limits (angle\*10)    |
| `setMaxAngles()`       | `{maxAngles...}`          | Set soft maximum angle limits (angle\*10)    |
| `setOffsets()`         | `{offsets...}`            | Set angle offsets for all servos             |
| `setInverts()`         | `{inverts...}`            | Set inversion flags for all servos           |
| `init()`               | -                         | Initialize the servo group                   |
| `init(Serial)`         | `Stream& debugStream`     | Initialize with debug logging                |

### 🎮 Control Methods

| Method           | Parameters                         | Description                                             |
| ---------------- | ---------------------------------- | ------------------------------------------------------- |
| `setPosition()`  | `servo_index`, `angle`, `duration` | Set individual servo position with smooth interpolation |
| `setPositions()` | `angles[]`, `duration`             | Set all servo positions simultaneously                  |
| `update()`       | -                                  | Update servo positions (call regularly in loop)         |
| `getState()`     | -                                  | Get current state: `IDLE`, `MOVING`, or `DETACHED`      |
| `getServoJson()` | -                                  | Get JSON string with current and goal positions         |
| `detach()`       | `servo_index`                      | Detach individual servo by stopping PWM signals         |
| `detachAll()`    | -                                  | Detach all servos by stopping PWM signals               |

#### 🔄 States

| State      | Description                                                   |
| ---------- | ------------------------------------------------------------- |
| `IDLE`     | All servos are at their target positions and not moving       |
| `MOVING`   | At least one servo is currently moving to its target position |
| `DETACHED` | All servos have been detached (PWM signals stopped)           |

## 📖 Examples

See the `examples/` folder for complete usage examples:

- `i2c_usage/` - I2C mode example using PCA9685
- `pwm_usage/` - Direct PWM mode example using GPIO pins

### 🤖 Real-World Usage

Check out [FoxMini](https://github.com/FoxLabsFr/FoxMini) - a 4-legged robot that uses the ServoGroup library for coordinated servo control.

## 📄 License

This project is licensed under Creative Commons Attribution-NonCommercial (CC BY-NC) with a special clause reserving all commercial rights to the original author. See LICENSE for details.

Built with ❤️ by FoxLabs

This README was generated with AI assistance.
