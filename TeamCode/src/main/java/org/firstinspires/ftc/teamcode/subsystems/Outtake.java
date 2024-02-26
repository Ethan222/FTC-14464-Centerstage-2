package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Outtake extends SubsystemBase {
    public static final String RETRACTED = "RETRACTED", PARTLY_EXTENDED = "PARTLY EXTENDED", EXTENDED = "EXTENDED", EXTRA_EXTENDED = "EXTRA EXTENDED";
    public static final String DOWN = "DOWN";

    public static class Claw extends SubsystemBase {
        private final ServoEx servo;
        public Claw(HardwareMap hardwareMap, String name, double upPsn, double downPsn) {
            servo = hardwareMap.get(ServoEx.class, name);
            if(downPsn < upPsn)
                servo.setRange(downPsn, upPsn);
            else {
                servo.setInverted(true);
                servo.setRange(upPsn, downPsn);
            }
        }
        public void down() {
            servo.setPosition(0);
        }
        public Action up() {
            return telemetryPacket -> {
                servo.setPosition(1);
                return false;
            };
        }
        public String getState() {
            double psn = servo.getPosition();
            if(psn < .05)
                return "DOWN";
            else if(psn > .95)
                return "UP";
            else return "PARTLY DOWN";
        }
        public String getTelemetry() {
            return String.format("%s (%.2f / %.2f deg)", getState(), servo.getPosition(), servo.getAngle());
        }
    }
    private final EncoderMotor motor;
    public ServoEx rotator;
    public Claw claw1, claw2;
    public static int[] presets = {0, 490, 750, 2000};
    public static double DEFAULT_ACCELERATION = .2;
    public static double EXTENDED_POS = .72;
    public int currentPreset = 0;
    public Outtake(HardwareMap hardwareMap, String motorName, String rotatorName, String claw1Name, String claw2Name) {
        motor = new EncoderMotor(hardwareMap, motorName, presets);
        motor.reverseDirection();
        motor.brakeOnZeroPower();
        rotator = hardwareMap.get(ServoEx.class, rotatorName);
        rotator.setRange(.4, 1);
        claw1 = new Claw(hardwareMap, claw1Name, .56, .78);
        claw2 = new Claw(hardwareMap, claw2Name, .8, 1);
    }
    public Action up(double power) {
        return motor.setPower(power);
    }
    public Action up() { return up(1); }
    public Action down(double power) {
        return motor.setPower(-power);
    }
    public Action down() { return down(1); }

    public Action accelerateUp(double accel) throws Exception {
        return motor.accelerateTo(1, accel);
    }
    public Action accelerateUp() throws Exception { return accelerateUp(DEFAULT_ACCELERATION); }
    public Action accelerateDown(double accel) throws Exception {
        return motor.accelerateTo(-1, accel);
    }
    public Action accelerateDown() throws Exception { return accelerateDown(DEFAULT_ACCELERATION); }
    public Action decelerate(double deceleration) throws Exception {
        return motor.accelerateTo(0, deceleration);
    }
    public Action decelerate() throws Exception {
        return decelerate(DEFAULT_ACCELERATION);
    }

    public Action rotate(double degrees) {
        return telemetryPacket -> {
            rotator.rotateByAngle(degrees);
            return false;
        };
    }
    public Action retract(double degrees) {
        return telemetryPacket -> {
            rotator.rotateByAngle(-degrees);
            return false;
        };
    }
    public Action rotate() {
        return telemetryPacket -> {
            rotator.setPosition(EXTENDED_POS);
            return false;
        };
    }
    public Action retract() {
        return telemetryPacket -> {
            rotator.setPosition(0);
            return false;
        };
    }

    public String getState() {
        int currentPreset = motor.getCurrentPreset();
        switch(currentPreset) {
            case 0:
                return DOWN;
            case -1:
                return null;
            default:
                return String.valueOf(currentPreset);
        }
    }
    public String getRotatorState() {
        double pos = rotator.getPosition();
        if(pos < .03)
            return RETRACTED;
        else if(Math.abs(pos - EXTENDED_POS) < .03)
            return EXTENDED;
        else if(pos < EXTENDED_POS)
            return PARTLY_EXTENDED;
        else return EXTRA_EXTENDED;
    }
    public String getMotorTelemetry() {
        return String.format("%s (%d) [%.1f]", getState(), motor.getPosition(), motor.getPower());
    }
    public String getRotatorTelemtry() {
        return String.format("%s (%.2f / %.2f deg)", getRotatorState(), rotator.getPosition(), rotator.getAngle());
    }
    public String getTelemetry() {
        return String.format("%s\n- rotator: %s \n- claw1: %s \n- claw2: %s", getMotorTelemetry(), getRotatorTelemtry(), claw1.getTelemetry(), claw2.getTelemetry());
    }
}