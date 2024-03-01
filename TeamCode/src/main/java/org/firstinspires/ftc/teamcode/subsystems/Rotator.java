package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Rotator extends CustomServo {
  private double CENTER_POS;
  private final double INCREMENT;
  public Rotator(String name, HardwareMap hardwareMap, String id, double minPos, double maxPos, double centerPos, double increment) {
    super(name, hardwareMap, id, minPos, maxPos);
    CENTER_POS = centerPos;
    INCREMENT = increment;
  }
  public void center() {
    setPosition(CENTER_POS);
  }
  public void setCenterPos() {
    CENTER_POS = getPosition();
  }
  @Override public double getIncrement() { return INCREMENT; }
}
