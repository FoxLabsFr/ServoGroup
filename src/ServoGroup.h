#ifndef SERVOGROUP_H
#define SERVOGROUP_H

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

class ServoGroup {
 public:
  enum class State { IDLE, MOVING, DETACHED };

  ServoGroup(uint8_t i2c_address, uint8_t num_servos);
  ~ServoGroup();
  void init(uint16_t min[], uint16_t max[], short offset[], int8_t invert[],
            uint8_t id[]);
  void setPosition(uint8_t servo_index, unsigned long delay, int16_t angle);
  void setPositions(int16_t angles[], unsigned long delay);
  void detach(uint8_t servo_index);
  void detachAll();
  void update();

  String getServoJson();
  String getPositionsJson();
  int16_t* getCurrentPositions();
  int16_t* getGoalPositions();
  State getState();
  uint8_t num_servos;

 private:
  Adafruit_PWMServoDriver pwm;
  uint8_t i2c_address;
  bool initialized = false;
  uint16_t* servoMin;
  uint16_t* servoMax;
  short* servoOffset;
  int16_t* position;
  int8_t* invert;
  int16_t* goalPosition;
  unsigned long lastUpdate;
  unsigned long* moveDuration;
  unsigned long* startTime;
  uint16_t* startTimePosition;
  uint8_t* servoId;
  State state;
  uint16_t angleToPulse(uint8_t servo_index, int16_t angle);
  void applyPosition(uint8_t servo_index, int16_t angle);
};

#endif