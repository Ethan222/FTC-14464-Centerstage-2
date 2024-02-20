package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor {
  protected DcMotorEx motor;
  private final static double DEFAULT_ACCELERATION = .2;
  private int[] positions;

  public Motor(HardwareMap hardwareMap, String name, int[] psns) {
    motor = hardwareMap.get(DcMotorEx.class, name);
    if(psns != null)
      motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    else
      motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    positions = psns;
  }
  public Motor(HardwareMap hardwareMap, String name) { this(hardwareMap, name, null); }

  public double getPower() {
    return motor.getPower();
  }
  
  public void setPower(double power) {
    motor.setPower(MathUtils.clamp(power, -1, 1));
  }
  public void changePower(double change) {
    setPower(getPower() + change);
  }
  public void stop() {
    setPower(0);
  }

  // acceleration 
  public Action accelerateTo(double power, double accel) throws Exception {
    power = MathUtils.clamp(power, -1, 1);
    double finalPower = power;
    if(accel == 0)
      throw new Exception("Acceleration can't be 0");
    return new Action() {
      private final double targetPower = finalPower, acceleration = accel;
      
      @Override
      public boolean run(@NonNull TelemetryPacket packet) {
        double power = getPower();
        packet.put("power", power);
        if(power != targetPower) {
          if(Math.abs(power - targetPower) < acceleration)
            setPower(targetPower);
          else
            changePower(power < targetPower ? acceleration : -acceleration);
          return true; // rerun action
        } else
          return false; // stop action
      }
    };
  }
  public Action accelerateTo(double power) throws Exception {
    return accelerateTo(power, DEFAULT_ACCELERATION);
  }
}
