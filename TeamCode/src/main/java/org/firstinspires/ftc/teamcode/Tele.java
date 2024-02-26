package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class Tele extends LinearOpMode {
    private static boolean fieldCentric = false;
    private static final double WHEEL_SLOW_SPEED = .3;
    private static Pose2d startPose = new Pose2d(0, 0, 0);

    private Robot robot;
    private MecanumDrive drive;
    private double speed = 1;
    private GamepadEx driver1, driver2;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap, startPose);
        Motor fL = new Motor(hardwareMap, org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.FL_NAME);
        Motor fR = new Motor(hardwareMap, org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.FR_NAME);
        Motor bL = new Motor(hardwareMap, org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.BL_NAME);
        Motor bR = new Motor(hardwareMap, org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.BR_NAME);
        drive = new MecanumDrive(fL, fR, bL, bR);
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        telemetry.addLine("Initialized");
        telemetry.addData("Drive field centric (RT/LT)", () -> fieldCentric);
        while (opModeInInit()) {
            if(gamepad1.right_trigger > .5)
                fieldCentric = true;
            else if(gamepad1.left_trigger > .5)
                fieldCentric = false;

            if(gamepad1.start && gamepad1.back)
                requestOpModeStop();

            telemetry.update();
        }

        telemetry.addData("Wheel speed (RT)", () -> speed);
        telemetry.addLine();
        telemetry.addData("intake", robot.intake::getTelemetry);
        telemetry.addData("outtake", robot.outtake::getTelemetry);
        telemetry.addData("pixel pusher", robot.pixelPusher::getTelemetry);
        telemetry.addData("hang", robot.hang::getTelemetry);
        while(opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            double leftX = speed * driver1.getLeftX(), leftY = speed * driver1.getLeftY(), rightX = speed * driver1.getRightX();
            if(fieldCentric)
                drive.driveFieldCentric(leftX, leftY, rightX, robot.drive.imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            else drive.driveRobotCentric(leftX, leftY, rightX);
            robot.drive.updatePoseEstimate();

            // single driver mode
            if(gamepad1.back && driver2 != driver1)
                driver2 = driver1;
            else if(driver2 == driver1)
                driver2 = new GamepadEx(gamepad2);

            if(!gamepad1.back) {
                // field centric
                if (gamepad1.right_trigger > .5)
                    fieldCentric = true;
                else if (gamepad1.left_trigger > .5)
                    fieldCentric = false;

                // slow mode
                if (gamepad1.right_trigger > .3)
                    speed = WHEEL_SLOW_SPEED;
                else if(speed != 1)
                    speed = 1;
            }

            // intake
            double rt = driver2.getTrigger(Trigger.RIGHT_TRIGGER), lt = driver2.getTrigger(Trigger.LEFT_TRIGGER);
            if(rt > 0) {
                runningActions.add(robot.intake.in());
                if(driver2.isDown(Button.BACK)) {
                    runningActions.add(rt > .8 ? robot.intake.lower() : robot.intake.lower(1));
                }
            } else if(lt > 0) {
                runningActions.add(robot.intake.out());
                if(driver2.isDown(Button.BACK))
                    runningActions.add(lt > .8 ? robot.intake.raise() : robot.intake.raise(1));
            } else if(robot.intake.getPower() != 0)
                runningActions.add(robot.intake.stop());


            // update running actions
            List<Action> newActions = new ArrayList<>();
            for(Action action : runningActions) {
                if(action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);
        }
    }
}
