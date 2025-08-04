#include "ServoGroup.h"

ServoGroup::ServoGroup() {
  // Initialize with default values, arrays will be allocated when setIds is called
  i2c_address = 0x40;
  num_servos = 0;
  mode = Mode::I2C;
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

void ServoGroup::init() {
  debugMode = false;
  debugStream = nullptr;
  
  if (num_servos == 0) {
    return;
  }
  
  if (mode == Mode::I2C) {
    Wire.begin();

    // Check if the PWM driver is connected
    Wire.beginTransmission(i2c_address);
    if (Wire.endTransmission() == 0) {
      // PWM driver found
    } else {
      return;
    }

    pwm.begin();
    pwm.setPWMFreq(60);
    pwm.setOscillatorFrequency(27000000);
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
}

void ServoGroup::init(Stream& debugStream) {
  debugMode = true;
  this->debugStream = &debugStream;
  
  if (num_servos == 0) {
    debugLog("Error: No servos configured. Call setIds() first!");
    return;
  }
  
  if (mode == Mode::I2C) {
    Wire.begin();

    // Check if the PWM driver is connected
    Wire.beginTransmission(i2c_address);
    if (Wire.endTransmission() == 0) {
      debugLog("PWM driver 0x" + String(i2c_address, HEX) + " found on I2C bus.");
    } else {
      debugLog("Error: PWM driver not found on I2C bus!");
      return;
    }

    pwm.begin();
    pwm.setPWMFreq(60);
    pwm.setOscillatorFrequency(27000000);
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

int16_t* ServoGroup::getCurrentPositions() { return position; }

int16_t* ServoGroup::getGoalPositions() { return goalPosition; }

ServoGroup::State ServoGroup::getState() { return state; }