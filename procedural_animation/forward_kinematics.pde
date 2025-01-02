class ForwardKinematics extends Algorithm {
  ArrayList<PVector> points;
  ArrayList<Integer> lengths;
  ArrayList<Float> initialAngles;
  ArrayList<Float> angles;
  
  PVector origin;
  
  int maxPoints;
  int segmentSize;
  int pointSize;
  float initAngle;
  float angleSpeed;
  
  void init() {
    points = new ArrayList<PVector>();
    lengths = new ArrayList<Integer>();
    angles = new ArrayList<Float>();
    initialAngles = new ArrayList<Float>();
    
    origin = new PVector(width / 2, height / 2);
    
    // Initialize initial angles
    initialAngles.add(0f);
    initialAngles.add(1.3f);
    initialAngles.add(1f);
    initialAngles.add(0f);
    
    // Initialize points and lengths
    points.add(origin.copy());
    maxPoints = 4;
    segmentSize = 100;
    pointSize = 25;
    for (int i = 1; i < maxPoints; i++) {
      PVector newPoint = PVector.add(points.get(i - 1), new PVector(0, segmentSize));
      points.add(newPoint);
      lengths.add(segmentSize); // All segments have the same length
    }
    
    // Initialize angles
    for (int i = 0; i < maxPoints; i++) {
      angles.add(0f);
    }
    
    initAngle = 0f;
    angleSpeed = 0.03f;
  }
  
  float simplifyAngle(float angle) {
    while (angle >= TWO_PI) {
      angle -= TWO_PI;
    }
    while (angle < 0) {
      angle += TWO_PI;
    }
    return angle;
  }
  
  void update() {
    angles.set(0, simplifyAngle(initAngle));
    
    for (int i = 1; i < maxPoints; i++) {
      float parentAngle = angles.get(i - 1);
      float relativeAngle = initialAngles.get(i);
      float totalAngle = simplifyAngle(parentAngle + relativeAngle);
      angles.set(i, totalAngle);
    }
    
    for (int i = 1; i < maxPoints; i++) {
      float currentAngle = angles.get(i - 1);
      PVector parentPoint = points.get(i - 1);
      float segmentLength = lengths.size() > (i - 1) ? lengths.get(i - 1) : segmentSize;
      PVector newPoint = new PVector(
        parentPoint.x + cos(currentAngle) * segmentLength,
        parentPoint.y + sin(currentAngle) * segmentLength
      );
      points.set(i, newPoint);
    }
    
    initAngle += angleSpeed;
    initAngle = simplifyAngle(initAngle);
  }
  
  void display() {
    stroke(248, 248, 242);
    strokeWeight(8);
    
    // Draw lines between points
    for (int i = 0; i < points.size() - 1; i++) {
      PVector startPoint = points.get(i);
      PVector endPoint = points.get(i + 1);
      line(startPoint.x, startPoint.y, endPoint.x, endPoint.y);
    }
    
    // Draw joints
    stroke(248, 248, 242);
    fill(248, 248, 242);
    for (int i = 0; i < points.size(); i++) {
      if (i == 0) {
        circle(points.get(i).x, points.get(i).y, pointSize);
      } else {
        noStroke();
        fill(98, 114, 164);
        circle(points.get(i).x, points.get(i).y, pointSize);
      }
    }
  }
}
