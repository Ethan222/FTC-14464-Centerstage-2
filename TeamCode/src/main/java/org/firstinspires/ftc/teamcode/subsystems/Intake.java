package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final Motor motor;
//    private final ServoEx lowerer;

    public Intake(HardwareMap hardwareMap, String motorName, String servoName) {
        motor = new Motor(hardwareMap, motorName, true);
//        lowerer = hardwareMap.get(ServoEx.class, servoName);
//        lowerer.setRange(.5, 1);
//        lowerer.setInverted(true);
    }

    public void in(double power) {
        motor.setPower(power);
    }
    public void in() { in(1); }
    public void out(double power) {
        motor.setPower(-power);
    }
    public void out() { out(1); }
    public void stop() {
        motor.stop();
    }
    public double getPower() {
        return motor.getPower();
    }

//    public Action lower(double degrees) {
//        return telemetryPacket -> {
//            lowerer.rotateByAngle(-degrees);
//            return false;
//        };
//    }
//    public Action raise(double degrees) {
//        return telemetryPacket -> {
//            lowerer.rotateByAngle(degrees);
//            return false;
//        };
//    }
//    public Action lower() {
//        return telemetryPacket -> {
//            lowerer.setPosition(0);
//            return false;
//        };
//    }
//    public Action raise() {
//        return telemetryPacket -> {
//            lowerer.setPosition(1);
//            return false;
//        };
//    }
//
//    public String getState() {
//        double psn = lowerer.getPosition();
//        if(psn < .03)
//            return "DOWN";
//        else if(psn > .97)
//            return "UP";
//        else return "UNSURE";
//    }

    public String getTelemetry() {
//        return String.format("%s (%.2f / %.2f deg) [%.1f]", getState(), lowerer.getPosition(), lowerer.getAngle(), motor.getPower());
        return String.format("%.1f", motor.getPower());
    }
}