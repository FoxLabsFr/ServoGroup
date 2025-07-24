#include <ServoGroup.h>

// Create servo group with I2C address 0x40 and 8 servos
ServoGroup servos(0x40, 8);

// Timing variables for non-blocking operation
unsigned long lastMoveTime = 0;
unsigned long lastPrintTime = 0;
int moveStep = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("ServoGroup Basic Example");
  
  // Define servo configuration arrays
  uint16_t minPositions[8] = {150, 150, 150, 150, 150, 150, 150, 150};
  uint16_t maxPositions[8] = {600, 600, 600, 600, 600, 600, 600, 600};
  short offsets[8] = {-137, 74, -117, -169, -6, 70, 22, 58};
  int8_t inversions[8] = {-1, 1, 1, -1, 1, -1, -1, 1};
  uint8_t servoIds[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  
  // Initialize the servo group
  servos.init(minPositions, maxPositions, offsets, inversions, servoIds);
  
  Serial.println("ServoGroup initialized successfully");
}

void loop() {
  // Update servo positions (call regularly - this is critical!)
  servos.update();
  
  unsigned long currentTime = millis();
  
  // Non-blocking movement sequence
  if (currentTime - lastMoveTime > 2000) { // Wait 2 seconds between moves
    switch(moveStep) {
      case 0:
        // Move all servos to center position
        Serial.println("Moving to center position");
        int16_t centerPositions[8] = {900, 900, 900, 900, 900, 900, 900, 900};
        servos.setPositions(centerPositions, 1000); // 1 second duration
        moveStep++;
        break;
        
      case 1:
        // Move to different positions
        Serial.println("Moving to position 1");
        int16_t positions1[8] = {800, 800, 800, 800, 800, 800, 800, 800};
        servos.setPositions(positions1, 500);
        moveStep++;
        break;
        
      case 2:
        // Move individual servo
        Serial.println("Moving individual servo");
        servos.setPosition(0, 1000, 1000); // Move servo 0 to position 1000 over 1 second
        moveStep++;
        break;
        
      case 3:
        // Reset to start
        Serial.println("Resetting sequence");
        moveStep = 0;
        break;
    }
    lastMoveTime = currentTime;
  }
  
  // Print current state every 500ms (non-blocking)
  if (currentTime - lastPrintTime > 500) {
    Serial.print("Servo state: ");
    switch(servos.getState()) {
      case ServoGroup::State::IDLE:
        Serial.println("IDLE");
        break;
      case ServoGroup::State::MOVING:
        Serial.println("MOVING");
        break;
      case ServoGroup::State::DETACHED:
        Serial.println("DETACHED");
        break;
    }
    lastPrintTime = currentTime;
  }
} 