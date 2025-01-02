final int NUM_ALGORITHMS = 5;
Algorithm[] algorithms;

int currentAlgorithm = 0;

String[] algorithmNames = {
  "Distance Constraint",
  "Forward Kinematics",
  "CCD IK",
  "Jacobian Inverse IK",
  "FABRIK IK"
};

void setup() {
  fullScreen();
  
  algorithms = new Algorithm[NUM_ALGORITHMS];
  
  algorithms[0] = new DistanceConstraint();
  algorithms[1] = new ForwardKinematics();
  algorithms[2] = new CCDIK();
  algorithms[3] = new JacobianInverseIK();
  algorithms[4] = new FABRIKIK();
  
  // Initialize the current algorithm
  algorithms[currentAlgorithm].init();
  PFont jetbrain;
  
  jetbrain = createFont("JetBrainsMono-Bold.ttf", 128);
  textFont(jetbrain);
  textAlign(CENTER, TOP);
  textSize(24);
  fill(248, 248, 242);
}

void draw() {
  background(40, 42, 54);
  
  algorithms[currentAlgorithm].update();
  algorithms[currentAlgorithm].display();
  
  // Display the algorithm name at the top-middle
  fill(248, 248, 242);
  text(algorithmNames[currentAlgorithm], width / 2, 100);
}

void mousePressed() {
  // Cycle to the next algorithm on mouse click
  currentAlgorithm = (currentAlgorithm + 1) % NUM_ALGORITHMS;
  algorithms[currentAlgorithm].init();
}
