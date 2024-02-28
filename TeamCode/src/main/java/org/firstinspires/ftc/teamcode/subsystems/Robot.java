package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive drive;
    public Intake intake;
    public Outtake outtake;

    public Robot(HardwareMap hardwareMap, Pose2d startPose) {
        intake = new Intake(hardwareMap, "motor0", "");
        outtake = new Outtake(hardwareMap, new String[]{"servo0", "servo1", "servo2", "servo3"});
    }
}
