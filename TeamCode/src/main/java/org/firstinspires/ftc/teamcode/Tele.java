package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

@TeleOp(group = "_tele")
public class Tele extends LinearOpMode {
    private static boolean fieldCentric = false;
    private static final double WHEEL_SLOW_SPEED = .3;
    private static Pose2d startPose = new Pose2d(0, 0, 0);
    private static Alliance alliance = Alliance.BLUE;

    private Robot robot;
    private MecanumDrive drive;
    private double speed = 1;
    private GamepadEx driver1, driver2;
    private TriggerReader rt;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private ScheduledExecutorService executorService;
    private boolean continueIntaking = false;
    public static void setStartPose(Pose2d pose) { startPose = pose; }
    private void initialize() {
        robot = new Robot(hardwareMap, startPose, telemetry);

        Motor fL = new Motor(hardwareMap, "FL", Motor.GoBILDA.RPM_435);
        Motor fR = new Motor(hardwareMap, "FR", Motor.GoBILDA.RPM_435);
        Motor bL = new Motor(hardwareMap, "BL", Motor.GoBILDA.RPM_435);
        Motor bR = new Motor(hardwareMap, "BR", Motor.GoBILDA.RPM_435);
        drive = new MecanumDrive(fL, fR, bL, bR);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        rt = new TriggerReader(driver2, Trigger.RIGHT_TRIGGER);

        executorService = Executors.newSingleThreadScheduledExecutor();

        alliance = Auto.getAlliance();

        robot.outtake.releaser.close();
    }

    @Override public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        initialize();

        telemetry.addLine("Initialized");
//        telemetry.addData("field centric (RB/LB)", () -> fieldCentric);
//        telemetry.addData("alliance (x/b)", () -> alliance);
//        telemetry.addData("start pos", poseToString(startPose));
        while (opModeInInit()) {
//            if(gamepad1.right_bumper)
//                fieldCentric = true;
//            else if(gamepad1.left_bumper)
//                fieldCentric = false;

            if((gamepad1.start && gamepad2.back) || (gamepad2.start && gamepad2.back))
                requestOpModeStop();

            telemetry.update();
        }
        telemetry.log().clear();

//        telemetry.addData("Wheel speed (RB)", () -> speed);
        telemetry.addData("intake (RT/LT)", robot.intake::getTelemetry);
        telemetry.addData("outtake", null).setRetained(true);
        telemetry.addData(" flipper (dpad up/down)", robot.outtake.flipper::getTelemetry);
        telemetry.addData(" extender (dpad up/down)", robot.outtake.extender::getTelemetry);
        telemetry.addData(" arm rotator (left stick x)", robot.outtake.armRotator::getTelemetry);
        telemetry.addData(" pixel rotator (right stick x)", robot.outtake.pixelRotator::getTelemetry);
        telemetry.addData(" releaser (a)", robot.outtake.releaser::getTelemetry);
        telemetry.addData("hang (back + RS)", robot.hang::getTelemetry);
//        telemetry.addData("launcher (RB/LB)", robot.launcher::getTelemetry);
        telemetry.addData("auto claw (back + x/y)", robot.autoClaw::getTelemetry);
        telemetry.addData("\nrunning actions len", runningActions::size);

        TelemetryPacket packet = new TelemetryPacket();
        while (opModeIsActive()) {
            getGamepadInput();

            List<Action> newActions = new ArrayList<>();
            for(Action action : runningActions) {
                if(action.run(packet))
                    newActions.add(action);
            }
            runningActions = newActions;

            telemetry.update();
        }
    }

    private static String poseToString(Pose2d startPose) {
        return String.format("(%.1f, %.1f) @ %.1f deg", startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble()));
    }

    private void getGamepadInput() {
        driver1.readButtons();
        driver2.readButtons();
        rt.readValue();

        if((gamepad1.start && gamepad1.back) || (gamepad2.start && gamepad2.back))
            requestOpModeStop();

        if(gamepad1.back && driver2 != driver1) driver2 = driver1;
        else if(driver2 == driver1) driver2 = new GamepadEx(gamepad2);

        // drive
        if(!gamepad1.back) {
            if(gamepad1.right_bumper)
                fieldCentric = true;
            else if(gamepad1.left_bumper)
                fieldCentric = false;
            if(gamepad1.x) alliance = Alliance.BLUE;
            else if(gamepad1.b) alliance = Alliance.RED;
        }
        double rotation = 0;
        if(fieldCentric)
            rotation = (robot.drive.pose.heading.toDouble() - Math.PI / 2) * (alliance == Alliance.BLUE ? 1 : -1);
        robot.drive.setDrivePowers(new PoseVelocity2d(
            rotate(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), rotation),
            -gamepad1.right_stick_x
        ));
        robot.drive.updatePoseEstimate();

        // intake
        if(rt.wasJustPressed()) {
            runningActions.add(robot.prepareForIntake());
        }
        if(driver2.getTrigger(Trigger.RIGHT_TRIGGER) > 0) {
            robot.intake.in(driver2.getTrigger(Trigger.RIGHT_TRIGGER));
            if(driver2.wasJustPressed(Button.START))
                continueIntaking = true;
        } else if(driver2.getTrigger(Trigger.LEFT_TRIGGER) > 0) {
            robot.intake.out(driver2.getTrigger(Trigger.LEFT_TRIGGER));
            continueIntaking = false;
        } else if(!continueIntaking)
            robot.intake.stop();

        // outtake
        if(gamepad2.back) {
            if (gamepad2.dpad_up)
                robot.outtake.flipper.rotateIncrementally();
            else if (gamepad2.dpad_down)
                robot.outtake.flipper.unrotateIncrementally();
            robot.outtake.motor.setPower(-gamepad2.left_stick_y);
        } else {
            if (driver2.wasJustPressed(Button.DPAD_UP)) {
                if(Objects.equals(robot.outtake.flipper.getState(), Flipper.UP))
                    robot.outtake.extender.goToMaxPos();
                else {
                    robot.outtake.releaser.close();
                    runningActions.add(robot.outtake.raise());
                }
            } else if (driver2.wasJustPressed(Button.DPAD_DOWN))
                runningActions.add(robot.outtake.lower());
            if (Math.abs(gamepad2.left_stick_y) > Math.abs(gamepad2.left_stick_x))
                robot.outtake.flipper.rotateBy(-gamepad2.left_stick_y / 300);
            if (Math.abs(gamepad2.right_stick_y) > Math.abs(gamepad2.right_stick_x))
                robot.outtake.extender.rotateBy(-gamepad2.right_stick_y / 100);
        }
        if (Math.abs(gamepad2.left_stick_x) > Math.abs(gamepad2.left_stick_y))
            robot.outtake.armRotator.rotateBy(gamepad2.left_stick_x / 200);
//        else if(robot.outtake.flipper.getState() == Flipper.UP)
//            robot.outtake.armRotator.rotateBy(0);
        if(Math.abs(gamepad2.right_stick_x) > Math.abs(gamepad2.right_stick_y))
            robot.outtake.pixelRotator.rotateBy(gamepad2.right_stick_x / 100);
        if(driver2.wasJustPressed(Button.LEFT_STICK_BUTTON)) {
            if(driver2.isDown(Button.START)) robot.outtake.armRotator.setCenterPos();
            else robot.outtake.armRotator.center();
        }
        if(driver2.wasJustPressed(Button.RIGHT_STICK_BUTTON)) {
            if(driver2.isDown(Button.START)) robot.outtake.pixelRotator.setCenterPos();
            else robot.outtake.pixelRotator.center();
        }
        if(!driver2.isDown(Button.DPAD_UP) && !driver2.isDown(Button.DPAD_DOWN)) {
            if(driver2.isDown(Button.DPAD_RIGHT)) robot.outtake.moveRight();
            else if(driver2.isDown(Button.DPAD_LEFT)) robot.outtake.moveLeft();
        }
        if(driver2.wasJustPressed(Button.A) && !driver2.isDown(Button.START)) robot.outtake.releaser.open();
        else if(driver2.wasJustPressed(Button.B) && !driver2.isDown(Button.START)) robot.outtake.releaser.close();

        // hang
        if(driver2.isDown(Button.BACK))
            robot.hang.setPower(-driver2.getRightY());

        // launcher
//        if(driver2.isDown(Button.RIGHT_BUMPER)) robot.launcher.rotate();
//        else if(driver2.isDown(Button.LEFT_BUMPER)) robot.launcher.unrotate();

        // auto claw
        if(gamepad2.x && gamepad2.back)
            robot.autoClaw.rotateIncrementally();
        else if(gamepad2.y && gamepad2.back)
            robot.autoClaw.unrotateIncrementally();

        // clear running actions
        if((gamepad1.start && gamepad1.x) || (gamepad2.start && gamepad2.x))
            runningActions.clear();
    }

    private Vector2d rotate(Vector2d orig, double rotation) {
        if(rotation == 0) return orig;
//        double magnitude = Math.abs(Math.pow(orig.x, 2) + Math.pow(orig.y, 2));
//        double angle = Math.atan(orig.y / orig.x) + rotation;
//        return new Vector2d(magnitude * Math.cos(angle), magnitude * Math.sin(angle));
        double newX = (orig.x * Math.cos(rotation)) - (orig.y * Math.sin(rotation));
        double newY = (orig.x * Math.sin(rotation)) + (orig.y * Math.cos(rotation));
        return new Vector2d(newX, newY);
    }
}
