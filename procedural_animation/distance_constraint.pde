class DistanceConstraint extends Algorithm {
  ArrayList<PVector> joints;
  int segmentSize;
  int jointSize;
  PVector originPoint;
  int jointCount;

  void init() {
    joints = new ArrayList<PVector>();
    originPoint = new PVector(width / 2, height / 2);
    jointCount = 5;
    segmentSize = 100;
    jointSize = 10;
    joints.add(originPoint.copy());
    for (int i = 1; i < jointCount; i++) {
      joints.add(PVector.add(joints.get(i - 1), new PVector(0, segmentSize)));
    }
  }

  void update() {
    for (int i = 1; i < jointCount; i++) {
      PVector current = joints.get(i);
      PVector prev = joints.get(i - 1);
      PVector dir = PVector.sub(current, prev);
      dir.setMag(segmentSize);
      joints.set(i, PVector.add(prev, dir));
    }

    joints.set(0, new PVector(mouseX, mouseY));
  }

  void display() {
    stroke(248, 248, 242);
    strokeWeight(8);

    // Draw constraint circles
    noFill();
    for (int i = 0; i < jointCount; i++) {
      circle(joints.get(i).x, joints.get(i).y, segmentSize * 2);
    }

    // Draw lines between joints
    for (int i = 0; i < joints.size() - 1; i++) {
      PVector startPoint = joints.get(i);
      PVector endPoint = joints.get(i + 1);
      line(startPoint.x, startPoint.y, endPoint.x, endPoint.y);
    }

    // Draw joints
    fill(248, 248, 242);
    noStroke();
    for (PVector joint : joints) {
      circle(joint.x, joint.y, jointSize);
    }
  }
}
