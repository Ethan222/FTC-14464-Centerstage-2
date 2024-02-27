package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "0")
public class Tele extends LinearOpMode {
    private static boolean fieldCentric = false;
    private static final double WHEEL_SLOW_SPEED = .3;
    private static Pose2d startPose = new Pose2d(0, 0, 0);

    private Robot robot;
//    private MecanumDrive drive;
    private double speed = 1;
    private GamepadEx driver1, driver2;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private boolean initialize() {
        robot = new Robot(hardwareMap, startPose);
//        Motor fL = new Motor(hardwareMap, org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.FL_NAME);
//        Motor fR = new Motor(hardwareMap, org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.FR_NAME);
//        Motor bL = new Motor(hardwareMap, org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.BL_NAME);
//        Motor bR = new Motor(hardwareMap, org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.BR_NAME);
//        drive = new MecanumDrive(fL, fR, bL, bR);
//
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        return true;
    }

    @Override public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        initialize();

        telemetry.addLine("Initialized");
//        telemetry.addData("Drive field centric (RT/LT)", () -> fieldCentric);
        while (opModeInInit()) {
//            if(gamepad1.right_trigger > .5)
//                fieldCentric = true;
//            else if(gamepad1.left_trigger > .5)
//                fieldCentric = false;

            if (gamepad1.start && gamepad1.back)
                requestOpModeStop();

            telemetry.update();
        }

//        telemetry.addData("Wheel speed (RB)", () -> speed);

        telemetry.addLine(robot.outtake.getTelemetry());

        double rotateAmount = .0005;
        while (opModeIsActive()) {
            if (gamepad2.a)
                robot.outtake.servos[0].rotateBy(rotateAmount);
            else if (gamepad2.b)
                robot.outtake.servos[0].rotateBy(-rotateAmount);
            if (gamepad2.y)
                robot.outtake.servos[1].rotateBy(rotateAmount);
            else if (gamepad2.x)
                robot.outtake.servos[1].rotateBy(-rotateAmount);
            if (gamepad2.right_bumper)
                robot.outtake.servos[2].rotateBy(rotateAmount);
            else if (gamepad2.left_bumper)
                robot.outtake.servos[2].rotateBy(-rotateAmount);
            if (gamepad2.right_trigger > .5)
                robot.outtake.servos[3].rotateBy(rotateAmount);
            else if (gamepad2.left_trigger > .5)
                robot.outtake.servos[3].rotateBy(-rotateAmount);

            telemetry.update();
        }
    }
}
