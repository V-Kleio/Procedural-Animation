class CCDIK extends Algorithm {
  int numJoints;
  float[] jointAngles;  // Joint angles (in radians)
  PVector[] jointPositions;  // Positions of the joints
  float[] lengths;  // Lengths of the segments
  PVector target;  // Target position (mouse position)
  
  void init() {
    numJoints = 5;
    jointAngles = new float[numJoints];
    jointPositions = new PVector[numJoints];
    lengths = new float[numJoints - 1];
    target = new PVector(mouseX, mouseY);
    
    // Initialize joint positions (start from the origin)
    jointPositions[0] = new PVector(width / 2, height / 2);
    for (int i = 1; i < numJoints; i++) {
      lengths[i - 1] = 100;  // Set segment lengths
      jointAngles[i - 1] = 0;
      jointPositions[i] = PVector.add(jointPositions[i - 1], new PVector(lengths[i - 1], 0));
    }
  }
  
  float normalizeAngle(float angle) {
    while (angle >= TWO_PI) angle -= TWO_PI;
    while (angle < 0) angle += TWO_PI;
    return angle;
  }
  
  void ccdIK(int iterations) {
    for (int iter = 0; iter < iterations; iter++) {
      for (int i = numJoints - 2; i >= 0; i--) {  // Start from the second last joint
        PVector currentEnd = jointPositions[numJoints - 1];
        PVector targetDir = PVector.sub(target, jointPositions[i]);
        PVector endDir = PVector.sub(currentEnd, jointPositions[i]);
        
        // Calculate the angles
        float angle = targetDir.heading() - endDir.heading();
        jointAngles[i] += angle;
        jointAngles[i] = normalizeAngle(jointAngles[i]);
        
        // Update positions of this joint and all downstream joints
        updateJointPositions();
        
        // Check if the end effector is close enough to the target
        if (PVector.dist(jointPositions[numJoints - 1], target) < 5) {
          return;
        }
      }
    }
  }
  
  void updateJointPositions() {
    for (int i = 1; i < numJoints; i++) {
      float angleSum = 0;
      for (int j = 0; j < i; j++) {
        angleSum += jointAngles[j];
      }
      jointPositions[i] = PVector.add(jointPositions[i - 1],
        new PVector(cos(angleSum) * lengths[i - 1], sin(angleSum) * lengths[i - 1]));
    }
  }
  
  void update() {
    target.set(mouseX, mouseY);
    ccdIK(10);  // Perform 10 iterations per frame
  }
  
  void display() {
    stroke(248, 248, 242);
    strokeWeight(8);
    
    // Draw lines between joints
    for (int i = 0; i < numJoints - 1; i++) {
      PVector startPoint = jointPositions[i];
      PVector endPoint = jointPositions[i + 1];
      line(startPoint.x, startPoint.y, endPoint.x, endPoint.y);
    }
    
    // Draw joints
    fill(248, 248, 242);
    noStroke();
    for (PVector joint : jointPositions) {
      ellipse(joint.x, joint.y, 25, 25);
    }
    
    // Draw the target
    fill(255, 85, 85);
    ellipse(target.x, target.y, 30, 30);
  }
}
