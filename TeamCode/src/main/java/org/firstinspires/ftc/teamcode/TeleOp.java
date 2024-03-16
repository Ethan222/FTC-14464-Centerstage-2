package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "_tele")
public class TeleOp extends LinearOpMode {
//    private static boolean fieldCentric = false;
    private static final double WHEEL_SLOW_SPEED = .4;
    private static Pose2d startPose = new Pose2d(0, 0, 0);
//    private static Alliance alliance = Alliance.BLUE;

    private Robot robot;
//    private MecanumDrive drive;
    private double speed = 1;
    private GamepadEx driver1, driver2;
    private TriggerReader rt1, rt2;
    private List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dash;
    private ElapsedTime loopTimer, armTimer;
    private boolean endgame;
    public static void setStartPose(Pose2d pose) { startPose = pose; }
    private void initialize() {
        robot = new Robot(hardwareMap, startPose);

//        Motor fL = new Motor(hardwareMap, "FL", Motor.GoBILDA.RPM_435);
//        Motor fR = new Motor(hardwareMap, "FR", Motor.GoBILDA.RPM_435);
//        Motor bL = new Motor(hardwareMap, "BL", Motor.GoBILDA.RPM_435);
//        Motor bR = new Motor(hardwareMap, "BR", Motor.GoBILDA.RPM_435);
//        drive = new MecanumDrive(fL, fR, bL, bR);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        rt1 = new TriggerReader(driver1, Trigger.RIGHT_TRIGGER);
        rt2 = new TriggerReader(driver2, Trigger.RIGHT_TRIGGER);

        dash = FtcDashboard.getInstance();
        loopTimer = new ElapsedTime();
        armTimer = new ElapsedTime();
        endgame = false;

//        alliance = Auto.getAlliance();
    }

    @Override public void runOpMode() {
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

        TelemetryPacket packet = new TelemetryPacket();
        while (opModeIsActive()) {
            getGamepadInput();

            List<Action> newActions = new ArrayList<>();
            for(Action action : runningActions) {
                if(action.run(packet))
                    newActions.add(action);
            }
            runningActions = newActions;

            updateTheTelemetry();
            telemetry.update();
            dash.sendTelemetryPacket(packet);
            loopTimer.reset();
        }
    }

    private void updateTheTelemetry() {
//        telemetry.addData("Wheel speed (RB)", () -> speed);
        if(!endgame) {
            telemetry.addData("intake (RT/LT)", robot.intake.getTelemetry());
            telemetry.addData("outtake", robot.outtake.motor.getTelemetry());
            telemetry.addData(" flipper (dpad up/down)", robot.outtake.flipper.getTelemetry());
            telemetry.addData(" extender (right stick y)", robot.outtake.extender.getTelemetry());
            telemetry.addData(" arm rotator (left stick x)", robot.outtake.armRotator.getTelemetry());
            telemetry.addData(" pixel rotator (right stick x)", robot.outtake.pixelRotator.getTelemetry());
            telemetry.addData(" releaser (a/b)", robot.outtake.releaser.getTelemetry());
            telemetry.addData("auto claw (x/y)", robot.autoClaw.getTelemetry());
            telemetry.addLine("\n switch to endgame: back + y");
        } else {
            telemetry.addLine("hang");
            telemetry.addLine(robot.hang.getTelemetry());
            telemetry.addData("\nlauncher (RT/LT)", robot.launcher.getTelemetry());
            telemetry.addLine("\n switch back to teleop: back + b");
        }
        telemetry.addData("loop time", "%.1f ms", loopTimer.milliseconds());
    }

    private static String poseToString(Pose2d startPose) {
        return String.format("(%.1f, %.1f) @ %.1f deg", startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble()));
    }

    private void getGamepadInput() {
        driver1.readButtons();
        driver2.readButtons();
        rt1.readValue();
        rt2.readValue();

        if((gamepad1.start && gamepad1.back) || (gamepad2.start && gamepad2.back))
            requestOpModeStop();

        if(gamepad1.back && driver2 != driver1) {
            driver2 = driver1;
//            rt2 = new TriggerReader(driver1, Trigger.RIGHT_TRIGGER);
        } else if(!gamepad1.back && driver2 == driver1) {
            driver2 = new GamepadEx(gamepad2);
//            rt2 = new TriggerReader(driver2, Trigger.RIGHT_TRIGGER);
        }

        // drive
        if(gamepad1.start || (!gamepad1.back && gamepad1.right_trigger > .5)) speed = WHEEL_SLOW_SPEED;
        else speed = 1;
//        double rotation = 0;
//        if(fieldCentric)
//            rotation = (robot.drive.pose.heading.toDouble() - Math.PI / 2) * (alliance == Alliance.BLUE ? 1 : -1);
        PoseVelocity2d vel;
        if(gamepad1.back && !rt1.isDown()) vel = new PoseVelocity2d(new Vector2d(0, 0), 0);
        else vel = new PoseVelocity2d(new Vector2d(speed * -gamepad1.left_stick_y, speed * -gamepad1.left_stick_x), speed * -gamepad1.right_stick_x);
        robot.drive.setDrivePowers(vel);
//        robot.drive.updatePoseEstimate();

        // teleop
        if(!endgame) {
            // intake
            if (rt2.wasJustPressed() || (gamepad1.back && rt1.wasJustPressed())) {
                runningActions.add(robot.prepareForIntake());
//            runningActions.add(robot.intake.inWithPeriodicOut());
            }
            double lt = driver2.getTrigger(Trigger.LEFT_TRIGGER), rt = driver2.getTrigger(Trigger.RIGHT_TRIGGER);
            if (lt > 0)
                robot.intake.out(lt);
            else if (rt > 0) {
//            robot.outtake.flipper.goToMinPos();
                robot.intake.in(rt);
            } else {
                robot.intake.stop();
//            cancel(Intake.InWithPeriodicOut.class);
            }

            // outtake
            if (gamepad2.back) {
                if (gamepad2.dpad_up)
                    robot.outtake.flipper.rotateIncrementally();
                else if (gamepad2.dpad_down)
                    robot.outtake.flipper.unrotateIncrementally();
                double lsy = gamepad2.left_stick_y;
                if (Math.abs(lsy) > Math.abs(gamepad2.left_stick_x))
                    robot.outtake.flipper.rotateBy(-lsy / 600);
            } else {
                if (driver2.isDown(Button.DPAD_UP) && !currentlyRunning(robot.outtake.getRaiseActionClass())) {
                    if (robot.outtake.flipper.getState().equals(Flipper.UP) && armTimer.seconds() > .5)
                        robot.outtake.extender.goToMaxPos();
                    else {
                        robot.outtake.releaser.close();
                        runningActions.clear();
                        cancel(robot.outtake.getLowerActionClass());
                        runningActions.add(robot.outtake.raise());
                        armTimer.reset();
                    }
                } else if (driver2.isDown(Button.DPAD_DOWN) && !currentlyRunning(robot.outtake.getLowerActionClass()))
                    runningActions.add(robot.outtake.lower());
                double lsy = driver2.getLeftY();
                if ((!gamepad1.back || !rt1.isDown()) && Math.abs(lsy) > Math.abs(driver2.getLeftX()) && !robot.outtake.flipper.getState().equals(Flipper.DOWN))
                    robot.outtake.motor.setPower(lsy);
                else robot.outtake.motor.stop();
                robot.outtake.extender.rotateBy(-gamepad2.right_stick_y / 300);
            }
            if (!driver2.isDown(Button.BACK)) {
                robot.outtake.armRotator.rotateBy(driver2.getLeftX() / 500);
                double rx = driver2.getRightX();
                if (Math.abs(rx) > Math.abs(driver2.getRightY()))
                    robot.outtake.pixelRotator.rotateBy(rx / 500);
            }
            if (driver2.wasJustPressed(Button.LEFT_STICK_BUTTON)) {
                if (driver2.isDown(Button.START)) robot.outtake.armRotator.setCenterPos();
                else robot.outtake.center();
            }
            if (driver2.wasJustPressed(Button.RIGHT_STICK_BUTTON)) {
                if (driver2.isDown(Button.START)) robot.outtake.pixelRotator.setCenterPos();
                else robot.outtake.pixelRotator.center();
            }
            if (driver2.isDown(Button.DPAD_RIGHT)) robot.outtake.moveRight();
            else if (driver2.isDown(Button.DPAD_LEFT)) robot.outtake.moveLeft();
            if (driver2.isDown(Button.A) && !driver2.isDown(Button.START) && !driver2.isDown(Button.BACK)) {
                if (gamepad2.back) robot.outtake.releaser.openIncrementally();
                else robot.outtake.releaser.open();
            } else if (driver2.isDown(Button.B) && !driver2.isDown(Button.START) && !driver2.isDown(Button.BACK)) {
                if (gamepad2.back) robot.outtake.releaser.closeIncrementally();
                else robot.outtake.releaser.close();
            }
        }

        // endgame
        if(gamepad2.back && (gamepad2.x || gamepad2.y)) endgame = true;
        else if(gamepad2.back && (gamepad2.a || gamepad2.b)) endgame = false;
        if(endgame) {
            // hang
            if(gamepad2.back) {
                robot.hang.leftBack.rotateBy(-gamepad2.left_stick_y / 1000);
                robot.hang.rightBack.rotateBy(-gamepad2.right_stick_y / 800);
                robot.hang.leftFront.set(-gamepad2.left_stick_x / 6);
                robot.hang.rightFront.set(gamepad2.right_stick_x / 5);
            } else {
                robot.hang.rotateBackServos(-gamepad2.left_stick_y / 900);
                robot.hang.rotateFrontServos(-gamepad2.left_stick_x / 5);
                robot.hang.motors.set(-gamepad2.right_stick_y);
            }
            if(gamepad2.left_bumper) robot.hang.lower();
            // launcher
            if(gamepad2.right_trigger > .5) robot.launcher.launch();
            else if(gamepad2.left_trigger > .5) robot.launcher.reset();
        }

        // auto claw
        if (driver2.isDown(Button.X) && !driver2.isDown(Button.START) && !driver2.isDown(Button.BACK))
            robot.autoClaw.down(.005);
        else if (driver2.isDown(Button.Y) && !driver2.isDown(Button.BACK))
            robot.autoClaw.up(.005);

        // clear running actions
        if((gamepad1.start && gamepad1.x) || (gamepad2.start && gamepad2.x))
            runningActions.clear();
    }

    private boolean currentlyRunning(Class<Action> targetAction) {
        for(Action action : runningActions) {
            if(action.getClass().equals(targetAction))
                return true;
        }
        return false;
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
    private void cancel(Class<Action> actionToCancel) {
        if(actionToCancel == null) return;
        for(int i = 0; i < runningActions.size(); i++) {
            if(runningActions.get(i).getClass().equals(actionToCancel))
                runningActions.remove(i);
        }
    }
}
