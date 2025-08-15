#ifndef SERVOGROUP_H
#define SERVOGROUP_H

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// Macro to create arrays from brace-enclosed lists (Arduino compatible)
#define SET_IDS(i2c, ...) do { \
  uint8_t ids[] = {__VA_ARGS__}; \
  setIds(i2c, ids, sizeof(ids)/sizeof(ids[0])); \
} while(0)

#define SET_IDS_PWM(...) do { \
  uint8_t pins[] = {__VA_ARGS__}; \
  setIds(pins, sizeof(pins)/sizeof(pins[0])); \
} while(0)

#define SET_DEFAULT_POSITION(...) do { \
  uint16_t positions[] = {__VA_ARGS__}; \
  setDefaultPosition(positions, sizeof(positions)/sizeof(positions[0])); \
} while(0)

#define SET_MIN_PULSE(...) do { \
  uint16_t minPulses[] = {__VA_ARGS__}; \
  setMinPulse(minPulses, sizeof(minPulses)/sizeof(minPulses[0])); \
} while(0)

#define SET_MAX_PULSE(...) do { \
  uint16_t maxPulses[] = {__VA_ARGS__}; \
  setMaxPulse(maxPulses, sizeof(maxPulses)/sizeof(maxPulses[0])); \
} while(0)

#define SET_MIN_ANGLES(...) do { \
  int16_t minAngles[] = {__VA_ARGS__}; \
  setMinAngles(minAngles, sizeof(minAngles)/sizeof(minAngles[0])); \
} while(0)

#define SET_MAX_ANGLES(...) do { \
  int16_t maxAngles[] = {__VA_ARGS__}; \
  setMaxAngles(maxAngles, sizeof(maxAngles)/sizeof(maxAngles[0])); \
} while(0)

#define SET_OFFSETS(...) do { \
  short offsets[] = {__VA_ARGS__}; \
  setOffsets(offsets, sizeof(offsets)/sizeof(offsets[0])); \
} while(0)

#define SET_INVERTS(...) do { \
  int8_t inverts[] = {__VA_ARGS__}; \
  setInverts(inverts, sizeof(inverts)/sizeof(inverts[0])); \
} while(0)

class ServoGroup {
 public:
  enum class State { IDLE, MOVING, DETACHED };
  enum class Mode { I2C, DIRECT_PWM };

  ServoGroup();
  ~ServoGroup();
  
  // Arduino compatible initialization methods
  void setIds(uint8_t i2c_address, uint8_t ids[], uint8_t count);
  void setIds(uint8_t pins[], uint8_t count); // For direct PWM mode
  void setDefaultPosition(uint16_t positions[], uint8_t count);
  void setMinPulse(uint16_t minPulses[], uint8_t count);
  void setMaxPulse(uint16_t maxPulses[], uint8_t count);
  void setMinAngles(int16_t minAngles[], uint8_t count);
  void setMaxAngles(int16_t maxAngles[], uint8_t count);
  void setOffsets(short offsets[], uint8_t count);
  void setInverts(int8_t inverts[], uint8_t count);
  
  // Template methods for brace-enclosed syntax (Arduino compatible)
  template<uint8_t N>
  void setIds(uint8_t i2c_address, const uint8_t (&ids)[N]) {
    setIds(i2c_address, const_cast<uint8_t*>(ids), N);
  }
  
  template<uint8_t N>
  void setIds(const char* mode, const uint8_t (&pins)[N]) {
    if (strcmp(mode, "pwm") == 0) {
      setIds(const_cast<uint8_t*>(pins), N);
    }
  }
  
  // Overload for int arrays (convert to uint8_t for PWM mode)
  template<uint8_t N>
  void setIds(const char* mode, const int (&pins)[N]) {
    if (strcmp(mode, "pwm") == 0) {
      uint8_t converted[N];
      for (uint8_t i = 0; i < N; i++) {
        converted[i] = static_cast<uint8_t>(pins[i]);
      }
      setIds(converted, N);
    }
  }
  
  template<uint8_t N>
  void setDefaultPosition(const uint16_t (&positions)[N]) {
    setDefaultPosition(const_cast<uint16_t*>(positions), N);
  }
  
  // Overload for int arrays (convert to uint16_t)
  template<uint8_t N>
  void setDefaultPosition(const int (&positions)[N]) {
    uint16_t converted[N];
    for (uint8_t i = 0; i < N; i++) {
      converted[i] = static_cast<uint16_t>(positions[i]);
    }
    setDefaultPosition(converted, N);
  }
  
  template<uint8_t N>
  void setMinPulse(const uint16_t (&minPulses)[N]) {
    setMinPulse(const_cast<uint16_t*>(minPulses), N);
  }
  
  // Overload for int arrays (convert to uint16_t)
  template<uint8_t N>
  void setMinPulse(const int (&minPulses)[N]) {
    uint16_t converted[N];
    for (uint8_t i = 0; i < N; i++) {
      converted[i] = static_cast<uint16_t>(minPulses[i]);
    }
    setMinPulse(converted, N);
  }
  
  template<uint8_t N>
  void setMaxPulse(const uint16_t (&maxPulses)[N]) {
    setMaxPulse(const_cast<uint16_t*>(maxPulses), N);
  }
  
  // Overload for int arrays (convert to uint16_t)
  template<uint8_t N>
  void setMaxPulse(const int (&maxPulses)[N]) {
    uint16_t converted[N];
    for (uint8_t i = 0; i < N; i++) {
      converted[i] = static_cast<uint16_t>(maxPulses[i]);
    }
    setMaxPulse(converted, N);
  }
  
  template<uint8_t N>
  void setMinAngles(const int16_t (&minAngles)[N]) {
    setMinAngles(const_cast<int16_t*>(minAngles), N);
  }
  
  template<uint8_t N>
  void setMaxAngles(const int16_t (&maxAngles)[N]) {
    setMaxAngles(const_cast<int16_t*>(maxAngles), N);
  }
  
  template<uint8_t N>
  void setOffsets(const short (&offsets)[N]) {
    setOffsets(const_cast<short*>(offsets), N);
  }
  
  // Overload for int arrays (convert to short)
  template<uint8_t N>
  void setOffsets(const int (&offsets)[N]) {
    short converted[N];
    for (uint8_t i = 0; i < N; i++) {
      converted[i] = static_cast<short>(offsets[i]);
    }
    setOffsets(converted, N);
  }
  
  template<uint8_t N>
  void setInverts(const int8_t (&inverts)[N]) {
    setInverts(const_cast<int8_t*>(inverts), N);
  }
  
  // Overload for int arrays (convert to int8_t)
  template<uint8_t N>
  void setInverts(const int (&inverts)[N]) {
    int8_t converted[N];
    for (uint8_t i = 0; i < N; i++) {
      converted[i] = static_cast<int8_t>(inverts[i]);
    }
    setInverts(converted, N);
  }
  
  void init();
  void init(Stream& debugStream);
            
  void setPosition(uint8_t servo_index, int16_t angle, unsigned long delay);
  void setPositions(int16_t angles[], unsigned long delay);
  void detach(uint8_t servo_index);
  void detachAll();
  void update();

#ifndef SERVOGROUP_DISABLE_JSON
  String getServoJson();
  String getPositionsJson();
#endif

  int16_t* getCurrentPositions();
  int16_t* getGoalPositions();
  State getState();
  uint8_t num_servos;

 private:
  Adafruit_PWMServoDriver pwm;
  uint8_t i2c_address;
  bool initialized = false;
  Mode mode = Mode::I2C;
  uint16_t* servoMinPulse;
  uint16_t* servoMaxPulse;
  int16_t* servoMinAngle;  // Soft angle limitation (angle*10)
  int16_t* servoMaxAngle;  // Soft angle limitation (angle*10)
  short* servoOffset;
  int16_t* position;
  int8_t* invert;
  int16_t* goalPosition;
  uint16_t* defaultPosition;
  unsigned long lastUpdate;
  unsigned long* moveDuration;
  unsigned long* startTime;
  uint16_t* startTimePosition;
  uint8_t* servoId;
  uint8_t* servoPins;
  Servo* servos;
  State state;
  bool debugMode = false;
  Stream* debugStream = nullptr;
  bool* detachedServos;  // Track which servos are detached
  uint16_t angleToPulse(uint8_t servo_index, int16_t angle);
  void applyPosition(uint8_t servo_index, int16_t angle);
  void allocateArrays(uint8_t num_servos);
  void debugLog(const String& message);
};

#endif