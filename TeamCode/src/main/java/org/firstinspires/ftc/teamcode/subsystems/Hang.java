package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Hang {
    public MotorGroup motors;
    public final CustomServo leftBack, rightBack;
    public final CRServo leftFront, rightFront;
    public Hang(HardwareMap hardwareMap, String leftMotor, String rightMotor, String leftBack, String rightBack, String leftFront, String rightFront) {
        Motor left = new Motor(hardwareMap, leftMotor), right = new Motor(hardwareMap, rightMotor);
        left.setInverted(true);
        motors = new MotorGroup(left, right);
        this.leftBack = new CustomServo(hardwareMap, leftBack, .4, .8);
        this.rightBack = new CustomServo(hardwareMap, rightBack, .3, .7);
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
        List<Double> motorSpeeds = motors.getSpeeds();
        return String.format("[%.2f, %.2f]", motorSpeeds.get(0), motorSpeeds.get(1));
    }
}
