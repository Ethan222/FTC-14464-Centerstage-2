package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hang {
    public Motor leftMotor, rightMotor;
    public final CustomServo leftBack, rightBack;
    public final CRServo leftFront, rightFront;
    public Hang(HardwareMap hardwareMap, String leftMotorId, String rightMotorId, String leftBack, String rightBack, String leftFront, String rightFront) {
        leftMotor = new Motor(hardwareMap, leftMotorId, true);
        rightMotor = new Motor(hardwareMap, rightMotorId);
        this.leftBack = new CustomServo(hardwareMap, leftBack, .4, .8);
        this.rightBack = new CustomServo(hardwareMap, rightBack, .3, .9);
        this.leftFront = new CRServo(hardwareMap, leftFront);
        this.rightFront = new CRServo(hardwareMap, rightFront);
    }
    public void raise() {
        leftBack.setPosition(.76);
        rightBack.setPosition(.65);
    }
    public void lower() {
        leftBack.setPosition(.44);
        rightBack.setPosition(.34);
    }

    public void rotateBackServos(double speed) {
        leftBack.rotateBy(speed);
        rightBack.rotateBy(speed);
    }
    public void rotateFrontServos(double speed) {
        leftFront.set(speed);
        rightFront.set(-speed);
    }

    public String getTelemetry() {
        return " back servos (LSY) \n" +
                "  left: " + leftBack.getTelemetry() + '\n' +
                "  right: " + rightBack.getTelemetry() + '\n' +
                " front servos (LSX) \n" +
                "  left: " + leftFront.get() + '\n' +
                "  right: " + rightFront.get() + '\n' +
                "motors (RSY): " + getMotorSpeedsAsString();
    }

    private String getMotorSpeedsAsString() {
        return String.format("[%s, %s]", leftMotor.getTelemetry(), rightMotor.getTelemetry());
    }

    public void setMotorPowers(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}
