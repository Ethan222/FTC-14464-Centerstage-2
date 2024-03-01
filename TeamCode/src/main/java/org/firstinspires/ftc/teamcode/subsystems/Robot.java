package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Robot {
    public org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive drive;
    public Intake intake;
    public Outtake outtake;
    public Hang hang;
    public Launcher launcher;
    public Robot(HardwareMap hardwareMap, Pose2d startPose) {
        this(hardwareMap);
        initDrive(hardwareMap, startPose);
    }
    public Robot(HardwareMap hardwareMap) {
        drive = null; // can't init drive w/o startPose
        intake = new Intake(hardwareMap, "intake", "");
        outtake = new Outtake(hardwareMap,"motor1","servo0","servo1","servo2","servo4","servo3");
        hang = new Hang(hardwareMap, "motor0");
        launcher = new Launcher(hardwareMap, "launcher");
    }
    public void initDrive(HardwareMap hardwareMap, Pose2d startPose) {
        drive = new MecanumDrive(hardwareMap, startPose);
    }

    public void prepareForIntake() {
        outtake.center();
        outtake.releaser.close();
    }
}
