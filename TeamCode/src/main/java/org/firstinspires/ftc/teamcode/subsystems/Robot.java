package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Robot {
    public org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive drive;
    public Intake intake;
    public Outtake outtake;

    public Robot(HardwareMap hardwareMap, Pose2d startPose) {
        drive = new MecanumDrive(hardwareMap, startPose);
        intake = new Intake(hardwareMap, "motor0", "");
        outtake = new Outtake(hardwareMap, "servo0", "servo1", "servo2", "servo3", "servo4");
    }
}
