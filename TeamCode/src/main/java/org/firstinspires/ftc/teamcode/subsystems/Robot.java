package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Robot {
    public MecanumDrive drive;
    public Intake intake;
    public Outtake outtake;
    public PixelPusher pixelPusher;
    public HangSubsystem hang;
    public Robot(HardwareMap hardwareMap, Pose2d startPose) {
        drive = new MecanumDrive(hardwareMap, startPose);
        intake = new Intake(hardwareMap, "intake", "servo4");
        outtake = new Outtake(hardwareMap, "armMotor", "outtakeRotator", "servo1", "servo2");
        pixelPusher = new PixelPusher(hardwareMap, "pixelPusher");
        hang = new HangSubsystem(hardwareMap, "hangMotor", "servo5");
    }
}