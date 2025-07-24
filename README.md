# ServoGroup Library

ServoGroup is an Arduino library for controlling multiple servos using an I2C-based PWM driver, such as the Adafruit PCA9685, or our [Servo_Controller](https://github.com/FoxLabsFr/Servo_Controller) board. It provides an easy-to-use interface for configuring and controlling servo motors in groups.

## 🚀 Features

- [x] **Multiple servo control** via I2C PWM drivers (PCA9685)
- [x] **Smooth motion interpolation** with configurable duration
- [x] **Position tracking** and state management
- [x] **Individual servo configuration** with min/max limits, offsets, and inversion
- [x] **Direct JSON status output** for easy integration with web interfaces
- [ ] **Support classic PWM servo**
- [ ] **Support mixed groups (I2C driven servo and classic PWM servo)**
- [ ] **Support multiple servo groups in one project with IDs**

## 🏗️ Hardware & Specifications

**Required Hardware:**

- **PCA9685 PWM Servo Driver** - Controls the servo motors via I2C communication
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

### 💡 Basic Example

The following example demonstrates how to use the ServoGroup library to control a robotic arm with 5 servos:

```cpp
#include "ServoGroup/ServoGroup.h"

ServoGroup robotArm;

void setup() {
  robotArm(0x40, 5); // Initialize with I2C address 0x40 and 5 servos

  uint8_t armId[5] = {0, 1, 2, 3, 4};
  uint16_t armMin[5] = {150, 150, 150, 150, 150};
  uint16_t armMax[5] = {600, 600, 600, 600, 600};
  short armOffset[5] = {-137, 74, -117, -169, -6};
  int8_t armInvert[5] = {-1, 1, 1, -1, 1};

  robotArm.init(armMin, armMax, armOffset, armInvert, armId);

  int16_t positions1[5] = {0, 1750, 1373, 950, 900};

  // Move all servos to positions1 in 2500ms
  robotArm.setPositions(positions1, 2500);
}

void loop() {
  robotArm.update();
}
```

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
  robotArm.update();
}
```

This approach ensures that your program remains responsive while performing periodic tasks.

## 📚 API Reference

### 🔧 Constructor

```cpp
ServoGroup(uint8_t i2c_address, uint8_t num_servos)
```

### ⚙️ Methods

| Method           | Parameters                                       | Description                                             |
| ---------------- | ------------------------------------------------ | ------------------------------------------------------- |
| `init()`         | `min[]`, `max[]`, `offset[]`, `invert[]`, `id[]` | Initialize the servo group with configuration arrays    |
| `setPosition()`  | `servo_index`, `duration`, `angle`               | Set individual servo position with smooth interpolation |
| `setPositions()` | `angles[]`, `duration`                           | Set all servo positions simultaneously                  |
| `update()`       | -                                                | Update servo positions (call regularly in loop)         |
| `getState()`     | -                                                | Get current state: `IDLE`, `MOVING`, or `DETACHED`      |
| `getServoJson()` | -                                                | Get JSON string with current and goal positions         |
| `detach()`       | `servo_index`                                    | Detach individual servo by stopping PWM signals         |
| `detachAll()`    | -                                                | Detach all servos by stopping PWM signals               |

#### 🔄 States

| State      | Description                                                   |
| ---------- | ------------------------------------------------------------- |
| `IDLE`     | All servos are at their target positions and not moving       |
| `MOVING`   | At least one servo is currently moving to its target position |
| `DETACHED` | All servos have been detached (PWM signals stopped)           |

## 📖 Examples

See the `examples/` folder for complete usage examples.

### 🤖 Real-World Usage

Check out [FoxMini](https://github.com/FoxLabsFr/FoxMini) - a 4-legged robot that uses the ServoGroup library for coordinated servo control.

## 📄 License

This project is licensed under Creative Commons Attribution-NonCommercial (CC BY-NC) with a special clause reserving all commercial rights to the original author. See LICENSE for details.

Built with ❤️ by FoxLabs

This README was generated with AI assistance.
