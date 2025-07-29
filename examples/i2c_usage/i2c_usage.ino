#include <ServoGroup.h>

// Create servo group for I2C mode
ServoGroup servoGroup1;

// Timing variables for non-blocking operation
unsigned long lastMoveTime = 0;
unsigned long lastPrintTime = 0;
int moveStep = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("ServoGroup I2C Mode Example");
  
  // I2C Mode (using PCA9685)
  servoGroup1.setIds(0x40, {0, 1, 2, 3, 4});
  servoGroup1.setDefaultPosition({0, 1750, 1373, 950, 900});
  
  // Optional config
  servoGroup1.setMinPulse({150, 150, 150, 150, 150});
  servoGroup1.setMaxPulse({600, 600, 600, 600, 600});
  servoGroup1.setOffsets({-137, 74, -117, -169, -6});
  servoGroup1.setInverts({-1, 1, 1, -1, 1});

  // Init servo at default position
  servoGroup1.init();
  
  Serial.println("ServoGroup I2C mode initialized successfully");
}

void loop() {
  // Update servo positions (call regularly - this is critical!)
  servoGroup1.update();
  
  unsigned long currentTime = millis();
  
  // Non-blocking movement sequence
  if (currentTime - lastMoveTime > 2000) { // Wait 2 seconds between moves
    switch(moveStep) {
      case 0:
        // Move all servos to center position
        Serial.println("Moving to center position");
        int16_t centerPositions[5] = {900, 900, 900, 900, 900};
        servoGroup1.setPositions(centerPositions, 1000); // 1 second duration
        moveStep++;
        break;
        
      case 1:
        // Move to different positions
        Serial.println("Moving to position 1");
        int16_t positions1[5] = {800, 800, 800, 800, 800};
        servoGroup1.setPositions(positions1, 500);
        moveStep++;
        break;
        
      case 2:
        // Move individual servo
        Serial.println("Moving individual servo");
        servoGroup1.setPosition(0, 1000, 1000); // Move servo 0 to position 1000 over 1 second
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
    switch(servoGroup1.getState()) {
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