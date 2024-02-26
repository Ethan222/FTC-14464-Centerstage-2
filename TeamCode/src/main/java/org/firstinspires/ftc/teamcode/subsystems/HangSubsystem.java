package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangSubsystem {
    public static final String DOWN = "DOWN", UP = "UP", IN_BETWEEN = "IN BETWEEN";
    private Motor motor;
    public ServoEx rotator;
    public HangSubsystem(HardwareMap hardwareMap, String motorName, String servoName) {
        motor = new Motor(hardwareMap, motorName);
        motor.reverseDirection();
        motor.brakeOnZeroPower();
        rotator = hardwareMap.get(ServoEx.class, servoName);
        rotator.setRange(.2, .5);
    }
    public Action up(double power) {
        return motor.setPower(power);
    }
    public Action down(double power) {
        return motor.setPower(-power);
    }
    public double getPower() { return motor.getPower(); }

    public Action rotateUp(double degrees) {
        return telemetryPacket -> {
            rotator.rotateByAngle(degrees);
            return false;
        };
    }
    public Action rotateDown(double degrees) {
        return telemetryPacket -> {
            rotator.rotateByAngle(-degrees);
            return false;
        };
    }
    public Action rotateUp() {
        return telemetryPacket -> {
            rotator.setPosition(1);
            return false;
        };
    }
    public Action rotateDown() {
        return telemetryPacket -> {
            rotator.setPosition(0);
            return false;
        };
    }

    public String getRotatorState() {
        double pos = rotator.getPosition();
        if(pos < .05)
            return DOWN;
        else if(pos > .95)
            return UP;
        else return IN_BETWEEN;
    }
    public String getTelemetry() {
        return String.format("%s (%.2f / %.2f deg) [%.1f]", getRotatorState(), rotator.getPosition(), rotator.getAngle(), motor.getPower());
    }
}