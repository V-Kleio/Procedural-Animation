class FABRIKIK extends Algorithm {
  int numJoints;
  PVector[] jointPositions;
  float[] lengths;
  PVector target;  // mouse position
  PVector origin;
  
  void init() {
    numJoints = 5;
    jointPositions = new PVector[numJoints];
    lengths = new float[numJoints - 1];
    target = new PVector(mouseX, mouseY);
    
    origin = new PVector(width / 2, height / 2);
    
    jointPositions[0] = origin.copy();
    for (int i = 1; i < numJoints; i++) {
      lengths[i - 1] = 100;  // All segments are 100 units long
      jointPositions[i] = PVector.add(jointPositions[i - 1], new PVector(lengths[i - 1], 0));
    }
  }
  
  void fabrikIK() {
    float epsilon = 1.0;  // Convergence threshold
    int maxIterations = 10;  // Limit iterations per frame
    float totalLength = 0;
    for (int i = 0; i < lengths.length; i++) {
      totalLength += lengths[i];
    }
    
    float distanceToTarget = PVector.dist(origin, target);
    
    if (distanceToTarget > totalLength) {
      // Target is unreachable; stretch towards the target
      for (int i = 0; i < numJoints - 1; i++) {
        float r = PVector.dist(target, jointPositions[i]);
        float lambda = lengths[i] / r;
        jointPositions[i + 1] = PVector.lerp(jointPositions[i], target, lambda);
      }
    } else {
      // Target is reachable; perform iterations of FABRIK
      for (int iter = 0; iter < maxIterations; iter++) {
        // Backward Reaching
        jointPositions[numJoints - 1] = target.copy();
        for (int i = numJoints - 2; i >= 0; i--) {
          PVector direction = PVector.sub(jointPositions[i], jointPositions[i + 1]);
          direction.normalize();
          direction.mult(lengths[i]);
          jointPositions[i] = PVector.add(jointPositions[i + 1], direction);
        }
        
        // Forward Reaching
        jointPositions[0] = origin.copy();
        for (int i = 1; i < numJoints; i++) {
          PVector direction = PVector.sub(jointPositions[i], jointPositions[i - 1]);
          direction.normalize();
          direction.mult(lengths[i - 1]);
          jointPositions[i] = PVector.add(jointPositions[i - 1], direction);
        }
        
        // Check for convergence
        if (PVector.dist(jointPositions[numJoints - 1], target) < epsilon) {
          break;
        }
      }
    }
  }
  
  void update() {
    target.set(mouseX, mouseY);
    fabrikIK();
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
    for (int i = 0; i < numJoints; i++) {
      circle(jointPositions[i].x, jointPositions[i].y, 25);
    }
    
    // Draw the target
    fill(255, 85, 85);
    ellipse(target.x, target.y, 30, 30);
  }
}
