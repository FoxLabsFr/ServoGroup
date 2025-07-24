#include "ServoGroup.h"

ServoGroup::ServoGroup(uint8_t i2c_address, uint8_t num_servos)
    : i2c_address(i2c_address), num_servos(num_servos), pwm(i2c_address) {
  servoMin = new uint16_t[num_servos];
  servoMax = new uint16_t[num_servos];
  servoOffset = new short[num_servos];
  position = new int16_t[num_servos];
  invert = new int8_t[num_servos];
  goalPosition = new int16_t[num_servos];
  moveDuration = new unsigned long[num_servos];
  startTime = new unsigned long[num_servos];
  startTimePosition = new uint16_t[num_servos];
  servoId = new uint8_t[num_servos];
  state = State::IDLE;
  initialized = false;  // Initialize the flag
}

ServoGroup::~ServoGroup() {
  delete[] servoMin;
  delete[] servoMax;
  delete[] servoOffset;
  delete[] position;
  delete[] invert;
  delete[] goalPosition;
  delete[] moveDuration;
  delete[] startTime;
  delete[] startTimePosition;
  delete[] servoId;
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
      servoMin[servo_index] +
      ((a * (servoMax[servo_index] - servoMin[servo_index])) / 1800);
  return pulse;
}

void ServoGroup::applyPosition(uint8_t servo_index, int16_t angle) {
  if (!initialized) {
    Serial.println("Error: ServoGroup not initialized!");
    return;
  }
  pwm.setPWM(servoId[servo_index], 0, angleToPulse(servo_index, angle));
}

void ServoGroup::init(uint16_t min[], uint16_t max[], short offset[],
                      int8_t invert[], uint8_t id[]) {
  Wire.begin();

  // Check if the PWM driver is connected
  Wire.beginTransmission(i2c_address);
  if (Wire.endTransmission() == 0) {
    Serial.println("PWM driver " + String(i2c_address, HEX) +
                   " found on I2C bus.");
  } else {
    Serial.println("Error: PWM driver not found on I2C bus!");
    return;
  }

  pwm.begin();
  pwm.setPWMFreq(60);
  pwm.setOscillatorFrequency(27000000);

  for (int i = 0; i < num_servos; i++) {
    servoMin[i] = min[i];
    servoMax[i] = max[i];
    servoOffset[i] = offset[i];
    this->invert[i] = invert[i];
    position[i] = servoMin[i];
    goalPosition[i] = servoMin[i];
    moveDuration[i] = 0;
    startTime[i] = -1;
    servoId[i] = id[i];
  }
  state = State::IDLE;
  lastUpdate = millis();
  initialized = true;
}

void ServoGroup::setPosition(uint8_t servo_index, unsigned long delay,
                             int16_t angle) {
  if (servo_index < num_servos && angle != (int16_t)-1) {
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
      goalPosition[i] = angles[i];
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
    // Set PWM to 0 to detach the servo (stops sending pulses)
    pwm.setPWM(servoId[servo_index], 0, 0);
    // Reset movement state for this servo
    startTime[servo_index] = 0;
    moveDuration[servo_index] = 0;
    // Update overall state if all servos are detached
    bool allDetached = true;
    for (int i = 0; i < num_servos; i++) {
      if (startTime[i] > 0) {
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
    // Set PWM to 0 to detach all servos
    pwm.setPWM(servoId[i], 0, 0);
    // Reset movement state
    startTime[i] = 0;
    moveDuration[i] = 0;
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