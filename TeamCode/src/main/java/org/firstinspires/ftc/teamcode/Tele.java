package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@TeleOp(group = "_tele")
public class Tele extends LinearOpMode {
    private static boolean fieldCentric = false;
    private static final double WHEEL_SLOW_SPEED = .3;
    private static Pose2d startPose = new Pose2d(0, 0, 0);

    private Robot robot;
//    private MecanumDrive drive;
    private double speed = 1;
    private GamepadEx driver1, driver2;
    private CustomButton a, b, x, y, dpadUp, dpadDown, rb, lb;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private ScheduledExecutorService executorService;
    private void initialize() {
        robot = new Robot(hardwareMap, startPose);
//        Motor fL = new Motor(hardwareMap, "FL");
//        Motor fR = new Motor(hardwareMap, "FR");
//        Motor bL = new Motor(hardwareMap, "BL");
//        Motor bR = new Motor(hardwareMap, "BR");
//        drive = new MecanumDrive(fL, fR, bL, bR);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        executorService = Executors.newSingleThreadScheduledExecutor();
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

            if((gamepad1.start && gamepad2.back) || (gamepad2.start && gamepad2.back))
                requestOpModeStop();

            telemetry.update();
        }
        telemetry.log().clear();

//        telemetry.addData("Wheel speed (RB)", () -> speed);
        telemetry.addData("\nintake (RT/LT)", robot.intake::getTelemetry);
        telemetry.addData("outtake", null).setRetained(true);
        telemetry.addData(" flipper (dpad up/down)", robot.outtake.flipper::getTelemetry);
        telemetry.addData(" extender (dpad up/down)", robot.outtake.extender::getTelemetry);
        telemetry.addData(" arm rotator (left stick x)", robot.outtake.armRotator::getTelemetry);
        telemetry.addData(" pixel rotator (right stick x)", robot.outtake.pixelRotator::getTelemetry);
        telemetry.addData(" releaser (a/b)", robot.outtake.pixelRotator::getTelemetry);

        while (opModeIsActive()) {
            if((gamepad1.start && gamepad2.back) || (gamepad2.start && gamepad2.back))
                requestOpModeStop();
            if(gamepad2.start && gamepad2.x)
                telemetry.log().clear();

            // drive
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            robot.drive.updatePoseEstimate();

            // intake
            if(gamepad2.right_trigger > 0)
                robot.intake.in(gamepad2.right_trigger);
            else if(gamepad2.left_trigger > 0)
                robot.intake.out(gamepad2.left_trigger);
            else robot.intake.stop();

            // outtake
            if(gamepad2.dpad_up) {
                robot.outtake.flipper.goToMaxPos();
                executorService.schedule(robot.outtake.extender::goToMaxPos, 200, TimeUnit.MILLISECONDS);
            } else if(gamepad2.dpad_down) {
                robot.outtake.extender.goToMinPos();
                robot.outtake.flipper.goToMinPos();
            }
            robot.outtake.flipper.rotateBy(-gamepad2.left_stick_y / 100);
            robot.outtake.extender.rotateBy(-gamepad2.right_stick_y / 100);
            robot.outtake.armRotator.rotateBy(gamepad2.left_stick_x / 500);
            robot.outtake.pixelRotator.rotateBy(gamepad2.right_stick_x / 100);
            if(gamepad2.a)
                robot.outtake.release();

//            List<Action> newActions = new ArrayList<>();
//            for(Action action : runningActions) {
//                if(action.run(new TelemetryPacket()))
//                    newActions.add(action);
//            }
//            runningActions = newActions;

            telemetry.update();
        }
    }
}
