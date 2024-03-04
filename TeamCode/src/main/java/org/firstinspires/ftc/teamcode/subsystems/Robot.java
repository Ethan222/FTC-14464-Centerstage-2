package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Robot {
    public org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive drive;
    public Intake intake;
    public Outtake outtake;
    public Hang hang;
    public Launcher launcher;
    public CustomServo autoClaw;
    public Robot(HardwareMap hardwareMap, Pose2d startPose, Telemetry telemetry) {
        this(hardwareMap, telemetry);
        initDrive(hardwareMap, startPose);
    }
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        drive = null; // can't init drive w/o startPose
        intake = new Intake(hardwareMap, "intake", "");
        outtake = new Outtake(hardwareMap,"motor1","servo0","servo1","servo2","servo4","servo3");
        hang = new Hang(hardwareMap, "motor0");
        try {
            launcher = new Launcher(hardwareMap, "launcher");
        } catch (Exception ignored) {}
        autoClaw = new CustomServo(hardwareMap, "auto");
    }
    public void initDrive(HardwareMap hardwareMap, Pose2d startPose) {
        drive = new MecanumDrive(hardwareMap, startPose);
    }

    public void prepareForIntake() {
        outtake.releaser.close();
    }
}
