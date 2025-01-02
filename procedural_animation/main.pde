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

  // Create all algorithm class and store it in an array
  algorithms = new Algorithm[NUM_ALGORITHMS];
  
  algorithms[0] = new DistanceConstraint();
  algorithms[1] = new ForwardKinematics();
  algorithms[2] = new CCDIK();
  algorithms[3] = new JacobianInverseIK();
  algorithms[4] = new FABRIKIK();
  
  algorithms[currentAlgorithm].init();

  // Setting for title
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
  
  // Update the title
  fill(248, 248, 242);
  text(algorithmNames[currentAlgorithm], width / 2, 100);
}

void mousePressed() {
  // Mouse click to change algorithm
  currentAlgorithm = (currentAlgorithm + 1) % NUM_ALGORITHMS;
  algorithms[currentAlgorithm].init();
}
