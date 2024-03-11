package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Robot {
    public org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive drive;
    public Intake intake;
    public Outtake outtake;
    public Hang hang;
    public Launcher launcher;
    public AutoClaw autoClaw;
    public Robot(HardwareMap hardwareMap, Pose2d startPose, Telemetry telemetry) {
        this(hardwareMap, telemetry);
        initDrive(hardwareMap, startPose);
    }
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        drive = null; // can't init drive w/o startPose
        intake = new Intake(hardwareMap, "intake");
        outtake = new Outtake(hardwareMap,"motor1","servo0","servo1","servo2","servo4","servo3");
        hang = new Hang(hardwareMap, "motor0");
        try {
            launcher = new Launcher(hardwareMap, "launcher");
        } catch (Exception ignored) {}
        autoClaw = new AutoClaw(hardwareMap, "auto");
    }
    public void initDrive(HardwareMap hardwareMap, Pose2d startPose) {
        drive = new MecanumDrive(hardwareMap, startPose);
    }

    public Action prepareForIntake() {
        outtake.releaser.close();
        return outtake.lower();
    }
    public static class AutoClaw extends CustomServo {
        public AutoClaw(HardwareMap hardwareMap, String id) {
            super(hardwareMap, id);
        }
        public void down(double inc) {
            rotateBy(-inc);
        }
        public void down() {
            setPosition(.15);
        }
        public void up(double inc) {
            rotateBy(inc);
        }
        public void up() {
            setPosition(.7);
        }
    }
}
