class JacobianInverseIK extends Algorithm {
  int numJoints; // Number of joints
  float[] jointAngles;
  PVector[] jointPositions;
  PVector target;  // mouse position
  float[] lengths;
  
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
  
  float[][] transpose(float[][] matrix) {
    int rows = matrix.length;
    int cols = matrix[0].length;
    float[][] transposed = new float[cols][rows];
    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        transposed[j][i] = matrix[i][j];
      }
    }
    return transposed;
  }
  
  void jacobianInverseIK() {
    float epsilon = 1.0;  // Convergence threshold
    float stepSize = 0.1;  // Step size for joint adjustments
    int maxIterations = 100;
    
    for (int iter = 0; iter < maxIterations; iter++) {
      // Calculate the end effector position
      PVector endEffector = jointPositions[numJoints - 1].copy();
      
      // Calculate the error vector
      PVector error = PVector.sub(target, endEffector);
      if (error.mag() < epsilon) {
        break;
      }
      
      // Calculate the Jacobian matrix
      float[][] jacobian = new float[2][numJoints];
      for (int i = 0; i < numJoints; i++) {
        PVector jointPos = jointPositions[i];
        PVector toEnd = PVector.sub(endEffector, jointPos);
        float dx = toEnd.x;
        float dy = toEnd.y;
        float distSq = dx * dx + dy * dy;
        if (distSq == 0) distSq = 0.0001f;  // Prevent division by zero
        
        // Partial derivatives
        jacobian[0][i] = -dy / distSq;
        jacobian[1][i] = dx / distSq;
      }
      
      // Compute Jacobian Transpose
      float[][] jacobianT = transpose(jacobian);
      
      // Compute delta angles (Jacobian Transpose * error)
      float[] deltaAngles = new float[numJoints];
      for (int i = 0; i < numJoints; i++) {
        deltaAngles[i] = jacobianT[i][0] * error.x + jacobianT[i][1] * error.y;
      }
      
      // Update joint angles
      for (int i = 0; i < numJoints; i++) {
        jointAngles[i] += stepSize * deltaAngles[i];
        jointAngles[i] = normalizeAngle(jointAngles[i]);
      }
      
      // Update joint positions based on new angles
      updateJointPositions();
    }
  }
  
  void updateJointPositions() {
    for (int i = 1; i < numJoints; i++) {
      float cumulativeAngle = 0;
      for (int j = 0; j < i; j++) {
        cumulativeAngle += jointAngles[j];
      }
      jointPositions[i] = PVector.add(jointPositions[i - 1],
        new PVector(cos(cumulativeAngle) * lengths[i - 1], sin(cumulativeAngle) * lengths[i - 1]));
    }
  }
  
  void update() {
    target.set(mouseX, mouseY);
    jacobianInverseIK();
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
    
    // Draw target
    fill(255, 85, 85);
    ellipse(target.x, target.y, 30, 30);
  }
}
