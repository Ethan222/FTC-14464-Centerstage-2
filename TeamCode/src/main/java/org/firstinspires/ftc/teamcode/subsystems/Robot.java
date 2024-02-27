package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive drive;
    public Outtake outtake;

    public Robot(HardwareMap hardwareMap, Pose2d startPose) {
        outtake = new Outtake(hardwareMap, new String[]{"servo0", "servo1", "servo2", "servo3"});
    }
}
