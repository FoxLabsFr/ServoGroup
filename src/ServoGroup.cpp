#include "ServoGroup.h"

ServoGroup::ServoGroup() {
  // Initialize with default values, arrays will be allocated when setIds is called
  i2c_address = 0x40;
  num_servos = 0;
  mode = Mode::I2C;
  custom_sda_pin = -1;
  custom_scl_pin = -1;
  servoMinPulse = nullptr;
  servoMaxPulse = nullptr;
  servoMinAngle = nullptr;
  servoMaxAngle = nullptr;
  servoOffset = nullptr;
  position = nullptr;
  invert = nullptr;
  goalPosition = nullptr;
  defaultPosition = nullptr;
  moveDuration = nullptr;
  startTime = nullptr;
  startTimePosition = nullptr;
  servoId = nullptr;
  servoPins = nullptr;
  servos = nullptr;
  detachedServos = nullptr;
  state = State::IDLE;
  initialized = false;
}

ServoGroup::~ServoGroup() {
  delete[] servoMinPulse;
  delete[] servoMaxPulse;
  delete[] servoMinAngle;
  delete[] servoMaxAngle;
  delete[] servoOffset;
  delete[] position;
  delete[] invert;
  delete[] goalPosition;
  delete[] defaultPosition;
  delete[] moveDuration;
  delete[] startTime;
  delete[] startTimePosition;
  delete[] servoId;
  delete[] servoPins;
  delete[] servos;
  delete[] detachedServos;
}

void ServoGroup::configureI2C(int sda_pin, int scl_pin) {
  custom_sda_pin = sda_pin;
  custom_scl_pin = scl_pin;
  
  if (debugMode && debugStream) {
    debugLog("I2C configuration: SDA=" + String(sda_pin) + ", SCL=" + String(scl_pin));
  }
}

void ServoGroup::allocateArrays(uint8_t num_servos) {
  this->num_servos = num_servos;
  
  // Allocate arrays
  servoId = new uint8_t[num_servos];
  position = new int16_t[num_servos];
  goalPosition = new int16_t[num_servos];
  servoMinPulse = new uint16_t[num_servos];
  servoMaxPulse = new uint16_t[num_servos];
  servoMinAngle = new int16_t[num_servos];
  servoMaxAngle = new int16_t[num_servos];
  servoOffset = new short[num_servos];
  invert = new int8_t[num_servos];
  moveDuration = new unsigned long[num_servos];
  startTime = new unsigned long[num_servos];
  startTimePosition = new uint16_t[num_servos];
  defaultPosition = new uint16_t[num_servos];
  servoPins = new uint8_t[num_servos];
  servos = new Servo[num_servos];
  detachedServos = new bool[num_servos];
  
  // Initialize arrays
  for (int i = 0; i < num_servos; i++) {
    servoId[i] = 0;
    position[i] = 0;
    goalPosition[i] = 0;
    servoMinPulse[i] = 150;
    servoMaxPulse[i] = 600;
    servoMinAngle[i] = 0;      // Default: full range (0°)
    servoMaxAngle[i] = 1800;   // Default: full range (180°)
    servoOffset[i] = 0;
    invert[i] = 1;
    moveDuration[i] = 0;
    startTime[i] = 0;
    startTimePosition[i] = 0;
    defaultPosition[i] = 0;
    servoPins[i] = 0;
    detachedServos[i] = false;
  }
}

void ServoGroup::setIds(uint8_t i2c_address, uint8_t ids[], uint8_t count) {
  mode = Mode::I2C;
  this->i2c_address = i2c_address;
  
  // Re-initialize PWM driver with correct I2C address
  pwm = Adafruit_PWMServoDriver(i2c_address);
  
  // Allocate arrays if not already done
  if (num_servos == 0) {
    allocateArrays(count);
  }
  
  // Copy servo IDs
  for (int i = 0; i < num_servos; i++) {
    servoId[i] = ids[i];
  }
}

void ServoGroup::setIds(uint8_t pins[], uint8_t count) {
  mode = Mode::DIRECT_PWM;
  
  // Allocate arrays if not already done
  if (num_servos == 0) {
    allocateArrays(count);
  }
  
  // Copy servo pins
  for (int i = 0; i < num_servos; i++) {
    servoPins[i] = pins[i];
  }
}

void ServoGroup::setDefaultPosition(uint16_t positions[], uint8_t count) {
  if (num_servos == 0) return;
  
  for (int i = 0; i < num_servos && i < count; i++) {
    defaultPosition[i] = positions[i];
  }
}

void ServoGroup::setMinPulse(uint16_t minPulses[], uint8_t count) {
  if (num_servos == 0) return;
  
  for (int i = 0; i < num_servos && i < count; i++) {
    servoMinPulse[i] = minPulses[i];
  }
}

void ServoGroup::setMaxPulse(uint16_t maxPulses[], uint8_t count) {
  if (num_servos == 0) return;
  
  for (int i = 0; i < num_servos && i < count; i++) {
    servoMaxPulse[i] = maxPulses[i];
  }
}

void ServoGroup::setMinAngles(int16_t minAngles[], uint8_t count) {
  if (num_servos == 0) return;
  
  for (int i = 0; i < num_servos && i < count; i++) {
    servoMinAngle[i] = minAngles[i];
  }
}

void ServoGroup::setMaxAngles(int16_t maxAngles[], uint8_t count) {
  if (num_servos == 0) return;
  
  for (int i = 0; i < num_servos && i < count; i++) {
    servoMaxAngle[i] = maxAngles[i];
  }
}

void ServoGroup::setOffsets(short offsets[], uint8_t count) {
  if (num_servos == 0) return;
  
  for (int i = 0; i < num_servos && i < count; i++) {
    servoOffset[i] = offsets[i];
  }
}

void ServoGroup::setInverts(int8_t inverts[], uint8_t count) {
  if (num_servos == 0) return;
  
  for (int i = 0; i < num_servos && i < count; i++) {
    invert[i] = inverts[i];
  }
}

void ServoGroup::debugLog(const String& message) {
  if (debugMode && debugStream) {
    // Simple safety check - just try to write
    debugStream->println(message);
  }
}

bool ServoGroup::init() {
  debugMode = false;
  debugStream = nullptr;
  
  if (num_servos == 0) {
    state = State::ERROR;
    return false;
  }
  
  // Reset error state if retrying initialization
  if (state == State::ERROR) {
    debugLog("Retrying initialization...");
    state = State::IDLE;
    initialized = false;
  }
  
  if (mode == Mode::I2C) {
    // Only initialize Wire once - it's a global singleton
    static bool wireInitialized = false;
    if (!wireInitialized) {
      bool i2c_success = false;
      
      if (custom_sda_pin != -1 && custom_scl_pin != -1) {
        // Use custom I2C pins (ESP32 specific)
        #ifdef ESP32
        i2c_success = Wire.begin(custom_sda_pin, custom_scl_pin);
        if (i2c_success) {
          debugLog("I2C initialized with custom pins: SDA=" + String(custom_sda_pin) + ", SCL=" + String(custom_scl_pin));
        } else {
          debugLog("Error: Failed to initialize I2C with custom pins: SDA=" + String(custom_sda_pin) + ", SCL=" + String(custom_scl_pin));
          state = State::ERROR;
          initialized = false;
          return false;
        }
        #else
        Wire.begin();
        i2c_success = true; // Assume success on non-ESP32 platforms
        debugLog("Warning: Custom I2C pins not supported on this platform, using default pins");
        #endif
      } else {
        // Use default I2C pins
        #ifdef ESP32
        i2c_success = Wire.begin();
        #else
        Wire.begin();
        i2c_success = true; // Assume success on non-ESP32 platforms
        #endif
        
        if (i2c_success) {
          debugLog("I2C initialized with default pins");
        } else {
          debugLog("Error: Failed to initialize I2C with default pins");
          state = State::ERROR;
          initialized = false;
          return false;
        }
      }
      wireInitialized = i2c_success;
    }

    // Check if the PWM driver is connected
    Wire.beginTransmission(i2c_address);
    if (Wire.endTransmission() == 0) {
      debugLog("PWM driver 0x" + String(i2c_address, HEX) + " found on I2C bus.");
      
      // Only initialize PWM driver if module is actually connected
      // This prevents I2C read errors from pwm.begin(), setPWMFreq(), etc.
      pwm.begin();
      pwm.setPWMFreq(60);
      pwm.setOscillatorFrequency(27000000);
      
      debugLog("PWM driver configured successfully");
    } else {
      debugLog("Error: PWM driver 0x" + String(i2c_address, HEX) + " not found on I2C bus!");
      state = State::ERROR;
      initialized = false;
      return false;
    }
  } else {
    // Direct PWM mode
    for (int i = 0; i < num_servos; i++) {
      servos[i].attach(servoPins[i], servoMinPulse[i], servoMaxPulse[i]);
    }
  }

  // Initialize movement tracking arrays
  for (int i = 0; i < num_servos; i++) {
    moveDuration[i] = 0;
    startTime[i] = -1;
  }
  
  // Only set to IDLE and initialized if we're not in error state
  if (state != State::ERROR) {
    state = State::IDLE;
    lastUpdate = millis();
    initialized = true;
    
    // Move servos directly to their default positions (immediate, no interpolation)
    for (int i = 0; i < num_servos; i++) {
      position[i] = defaultPosition[i];
      goalPosition[i] = defaultPosition[i];
      moveDuration[i] = 0;
      startTime[i] = -1;
      detachedServos[i] = false;  // Reset detached state
      applyPosition(i, defaultPosition[i]);
    }
    
    return true; // Initialization successful
  } else {
    // In error state, just initialize internal arrays but don't mark as initialized
    for (int i = 0; i < num_servos; i++) {
      position[i] = defaultPosition[i];
      goalPosition[i] = defaultPosition[i];
      moveDuration[i] = 0;
      startTime[i] = -1;
      detachedServos[i] = false;
    }
    
    return false; // Initialization failed
  }
}

bool ServoGroup::init(Stream& debugStream) {
  debugMode = true;
  this->debugStream = &debugStream;
  
  if (num_servos == 0) {
    debugLog("Error: No servos configured. Call setIds() first!");
    state = State::ERROR;
    return false;
  }
  
  // Reset error state if retrying initialization
  if (state == State::ERROR) {
    debugLog("Retrying initialization...");
    state = State::IDLE;
    initialized = false;
  }
  
  // Call the regular init() method to do the main initialization
  return init();
}

bool ServoGroup::checkI2CConnection() {
  if (mode != Mode::I2C) {
    return true; // Not applicable for PWM mode
  }
  
  // Don't attempt I2C operations if we're already in error state due to I2C issues
  // This prevents crashes when I2C is not properly initialized
  if (state == State::ERROR) {
    debugLog("I2C connection check skipped: already in error state");
    return false;
  }
  
  // Safely attempt I2C communication
  Wire.beginTransmission(i2c_address);
  uint8_t error = Wire.endTransmission();
  bool connected = (error == 0);
  
  if (debugMode) {
    if (connected) {
      debugLog("I2C connection check: PWM driver 0x" + String(i2c_address, HEX) + " is available");
    } else {
      debugLog("I2C connection check: PWM driver 0x" + String(i2c_address, HEX) + " is not available (error: " + String(error) + ")");
    }
  }
  
  return connected;
}


uint16_t ServoGroup::angleToPulse(uint8_t servo_index, int16_t angle) {
  short a = angle;
  if (invert[servo_index] == -1) {
    a = 1800 - angle;
    a -= servoOffset[servo_index];
  } else {
    a += servoOffset[servo_index];
  }

  // Perform the math directly instead of using map
  uint16_t pulse =
      servoMinPulse[servo_index] +
      ((a * (servoMaxPulse[servo_index] - servoMinPulse[servo_index])) / 1800);
  return pulse;
}

void ServoGroup::applyPosition(uint8_t servo_index, int16_t angle) {
  if (!initialized) {
    debugLog("Error: ServoGroup not initialized!");
    return;
  }
  
  if (state == State::ERROR) {
    debugLog("Error: ServoGroup in error state. Check I2C connection or call retryInit()");
    return;
  }
  
  if (mode == Mode::I2C) {
    pwm.setPWM(servoId[servo_index], 0, angleToPulse(servo_index, angle));
  } else {
    // Direct PWM mode - convert from angle*10 to degrees with rounding
    // This is a poor fix for problems with .writeMicroseconds()
    int degrees = (angle + 5) / 10;
    servos[servo_index].write(degrees);
  }
}

void ServoGroup::setPosition(uint8_t servo_index, int16_t angle, unsigned long delay) {
  if (servo_index < num_servos && angle != (int16_t)-1) {
    // Check if servo was previously detached and reattach if needed
    if (detachedServos[servo_index]) {
      if (mode == Mode::DIRECT_PWM) {
        // Reattach servo for PWM mode
        debugLog("Reattaching servo " + String(servo_index) + " (PWM mode)");
        servos[servo_index].attach(servoPins[servo_index], servoMinPulse[servo_index], servoMaxPulse[servo_index]);
      } else {
        // For I2C mode, no reattachment needed - just start sending PWM signals again
        debugLog("Reattaching servo " + String(servo_index) + " (I2C mode)");
      }
    }
    
    // Reset detached state
    detachedServos[servo_index] = false;
    
    // Apply soft angle limits
    if (angle < servoMinAngle[servo_index]) {
      angle = servoMinAngle[servo_index];
    }
    if (angle > servoMaxAngle[servo_index]) {
      angle = servoMaxAngle[servo_index];
    }
    
    goalPosition[servo_index] = angle;
    if (delay == 0) {
      position[servo_index] = goalPosition[servo_index];
      applyPosition(servo_index, goalPosition[servo_index]);
      startTime[servo_index] = -1;
      state = State::IDLE;
    } else {
      startTimePosition[servo_index] = position[servo_index];
      moveDuration[servo_index] = delay;
      startTime[servo_index] = millis();
      state = State::MOVING;
    }
  }
}

void ServoGroup::setPositions(int16_t angles[], unsigned long delay) {
  for (int i = 0; i < num_servos; i++) {
    if (angles[i] != (int16_t)-1) {
      // Check if servo was previously detached and reattach if needed
      if (detachedServos[i]) {
        if (mode == Mode::DIRECT_PWM) {
          // Reattach servo for PWM mode
          debugLog("Reattaching servo " + String(i) + " (PWM mode)");
          servos[i].attach(servoPins[i], servoMinPulse[i], servoMaxPulse[i]);
        } else {
          // For I2C mode, no reattachment needed - just start sending PWM signals again
          debugLog("Reattaching servo " + String(i) + " (I2C mode)");
        }
      }
      
      // Reset detached state
      detachedServos[i] = false;
      
      // Apply soft angle limits
      int16_t angle = angles[i];
      if (angle < servoMinAngle[i]) {
        angle = servoMinAngle[i];
      }
      if (angle > servoMaxAngle[i]) {
        angle = servoMaxAngle[i];
      }
      
      goalPosition[i] = angle;
      if (delay == 0) {
        position[i] = goalPosition[i];
        applyPosition(i, goalPosition[i]);
        startTime[i] = -1;
        state = State::IDLE;
      } else {
        startTimePosition[i] = position[i];
        moveDuration[i] = delay;
        startTime[i] = millis();
        state = State::MOVING;
      }
    }
  }
}

void ServoGroup::detach(uint8_t servo_index) {
  if (servo_index < num_servos) {
    if (mode == Mode::I2C) {
      // Check if we're in error state before attempting I2C operations
      if (state == State::ERROR || !initialized) {
        debugLog("Cannot detach servo: ServoGroup in error state or not initialized");
        return;
      }
      // Set PWM to 0 to detach the servo (stops sending pulses)
      pwm.setPWM(servoId[servo_index], 0, 0);
    } else {
      // Direct PWM mode
      servos[servo_index].detach();
    }
    // Reset movement state for this servo
    startTime[servo_index] = 0;
    moveDuration[servo_index] = 0;
    detachedServos[servo_index] = true;
    // Update overall state if all servos are detached
    bool allDetached = true;
    for (int i = 0; i < num_servos; i++) {
      if (!detachedServos[i]) {
        allDetached = false;
        break;
      }
    }
    if (allDetached) {
      state = State::DETACHED;
    }
  }
}

void ServoGroup::detachAll() {
  for (int i = 0; i < num_servos; i++) {
    if (mode == Mode::I2C) {
      // Check if we're in error state before attempting I2C operations
      if (state == State::ERROR || !initialized) {
        debugLog("Cannot detach servos: ServoGroup in error state or not initialized");
        break; // Exit loop, don't attempt any I2C operations
      }
      // Set PWM to 0 to detach all servos
      pwm.setPWM(servoId[i], 0, 0);
    } else {
      // Direct PWM mode
      servos[i].detach();
    }
    // Reset movement state
    startTime[i] = 0;
    moveDuration[i] = 0;
    detachedServos[i] = true;
  }
  state = State::DETACHED;
}

// Lerp function
float lerp(float start, float end, float progress) {
  return start + (end - start) * progress;
}

void ServoGroup::update() {
  bool allIdle = true;
  // Update each servo in the arm
  for (int i = 0; i < num_servos; ++i) {
    if (startTime[i] <= 0) continue;

    // Calculate the elapsed time since the movement started
    unsigned long elapsedTime = millis() - startTime[i];
    if (elapsedTime < moveDuration[i]) {
      // Calculate the progress
      float progress = static_cast<float>(elapsedTime) / moveDuration[i];

      // Use lerp to calculate the new position based on the start position
      float newPosition = lerp(startTimePosition[i], goalPosition[i], progress);
      position[i] = newPosition;
      allIdle = false;  // At least one servo is still moving
    } else {
      // Movement duration has elapsed, ensure servo is at target position
      position[i] = goalPosition[i];
      // Reset movement start time
      startTime[i] = 0;
    }

    // position[i] = goalPosition[i];
    applyPosition(i, position[i]);
  }
  state = allIdle ? State::IDLE : State::MOVING;
}

#ifndef SERVOGROUP_DISABLE_JSON
String ServoGroup::getServoJson() {
  String json = "{\"type\":\"servogroup\",\"state\":\"";
  
  // Add state string
  switch(state) {
    case State::IDLE:
      json += "idle";
      break;
    case State::MOVING:
      json += "moving";
      break;
    case State::DETACHED:
      json += "detached";
      break;
    case State::ERROR:
      json += "error";
      break;
  }
  
  json += "\",\"servos\":[";
  for (int i = 0; i < num_servos; ++i) {
    json += "{\"current\":";
    json += position[i];
    json += ",\"goal\":";
    json += goalPosition[i];
    json += "}";
    if (i < num_servos - 1) json += ",";
  }
  json += "]}";
  return json;
}

String ServoGroup::getPositionsJson() {
  String json = "[";
  for (int i = 0; i < num_servos; ++i) {
    json += String(position[i]);
    if (i < num_servos - 1) {
      json += ",";
    }
  }
  json += "]";
  return json;
}
#endif

int16_t* ServoGroup::getCurrentPositions() { return position; }

int16_t* ServoGroup::getGoalPositions() { return goalPosition; }

ServoGroup::State ServoGroup::getState() { return state; }