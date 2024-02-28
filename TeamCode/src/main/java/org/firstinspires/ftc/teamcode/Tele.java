package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CustomServo;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;
import java.util.List;

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

    private boolean initialize() {
        robot = new Robot(hardwareMap, startPose);
//        Motor fL = new Motor(hardwareMap, "FL");
//        Motor fR = new Motor(hardwareMap, "FR");
//        Motor bL = new Motor(hardwareMap, "BL");
//        Motor bR = new Motor(hardwareMap, "BR");
//        drive = new MecanumDrive(fL, fR, bL, bR);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        robot.outtake.servos[0].setButtons(new CustomButton(driver2, Button.DPAD_RIGHT), new CustomButton(driver2, Button.DPAD_LEFT));
        robot.outtake.servos[1].setButtons(new CustomButton(driver2, Button.X), new CustomButton(driver2, Button.Y));
        robot.outtake.servos[2].setButtons(new CustomButton(driver2, Button.RIGHT_BUMPER), new CustomButton(driver2, Button.LEFT_BUMPER));
        robot.outtake.servos[3].setButtons(new CustomButton(driver2, Button.DPAD_UP), new CustomButton(driver2, Button.DPAD_DOWN));

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

            if((gamepad1.start && gamepad2.back) || (gamepad2.start && gamepad2.back))
                requestOpModeStop();

            telemetry.update();
        }
        telemetry.log().clear();

//        telemetry.addData("Wheel speed (RB)", () -> speed);

        telemetry.addData("intake (RT/LT)", robot.intake::getTelemetry);
        telemetry.addData("\nouttake", robot.outtake::getTelemetry);

        while (opModeIsActive()) {
            if((gamepad1.start && gamepad2.back) || (gamepad2.start && gamepad2.back))
                requestOpModeStop();
            if(gamepad2.start && gamepad2.x)
                telemetry.log().clear();

            for(CustomServo servo : robot.outtake.servos)
                servo.update();

            if(gamepad2.right_trigger > 0)
                robot.intake.in(gamepad2.right_trigger);
            else if(gamepad2.left_trigger > 0)
                robot.intake.out(gamepad2.left_trigger);
            else robot.intake.stop();

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
