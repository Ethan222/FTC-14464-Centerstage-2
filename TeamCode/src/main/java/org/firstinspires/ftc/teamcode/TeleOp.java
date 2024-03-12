package org.firstinspires.ftc.teamcode;

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
import java.util.Objects;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

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
    private ScheduledExecutorService executorService;
    private ElapsedTime loopTimer;
//    private boolean continueIntaking = false;
    public static void setStartPose(Pose2d pose) { startPose = pose; }
    private void initialize() {
        robot = new Robot(hardwareMap, startPose, telemetry);

//        Motor fL = new Motor(hardwareMap, "FL", Motor.GoBILDA.RPM_435);
//        Motor fR = new Motor(hardwareMap, "FR", Motor.GoBILDA.RPM_435);
//        Motor bL = new Motor(hardwareMap, "BL", Motor.GoBILDA.RPM_435);
//        Motor bR = new Motor(hardwareMap, "BR", Motor.GoBILDA.RPM_435);
//        drive = new MecanumDrive(fL, fR, bL, bR);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        rt1 = new TriggerReader(driver1, Trigger.RIGHT_TRIGGER);
        rt2 = new TriggerReader(driver2, Trigger.RIGHT_TRIGGER);

        executorService = Executors.newSingleThreadScheduledExecutor();
        loopTimer = new ElapsedTime();

//        alliance = Auto.getAlliance();
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
        telemetry.addData("outtake", robot.outtake.motor::getTelemetry);
        telemetry.addData(" flipper (dpad up/down)", robot.outtake.flipper::getTelemetry);
        telemetry.addData(" extender (right stick y)", robot.outtake.extender::getTelemetry);
        telemetry.addData(" arm rotator (left stick x)", robot.outtake.armRotator::getTelemetry);
        telemetry.addData(" pixel rotator (right stick x)", robot.outtake.pixelRotator::getTelemetry);
        telemetry.addData(" releaser (a/b)", robot.outtake.releaser::getTelemetry);
        telemetry.addData("hang (back + RS)", robot.hang::getTelemetry);
//        telemetry.addData("launcher (RB/LB)", robot.launcher::getTelemetry);
        telemetry.addData("auto claw (x/y)", robot.autoClaw::getTelemetry);
        telemetry.addLine().addData("\nloop time", "%.1f ms", loopTimer::milliseconds).addData("running actions len", runningActions::size);

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
            loopTimer.reset();
        }
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
            rt2 = new TriggerReader(driver1, Trigger.RIGHT_TRIGGER);
        } else if(driver2 == driver1) {
            driver2 = new GamepadEx(gamepad2);
            rt2 = new TriggerReader(driver2, Trigger.RIGHT_TRIGGER);
        }

        // drive
        if(!gamepad1.back) {
            if(gamepad1.start) speed = WHEEL_SLOW_SPEED;
            else speed = 1;
//            if(gamepad1.right_bumper)
//                fieldCentric = true;
//            else if(gamepad1.left_bumper)
//                fieldCentric = false;
//            if(gamepad1.x) alliance = Alliance.BLUE;
//            else if(gamepad1.b) alliance = Alliance.RED;
        }
//        double rotation = 0;
//        if(fieldCentric)
//            rotation = (robot.drive.pose.heading.toDouble() - Math.PI / 2) * (alliance == Alliance.BLUE ? 1 : -1);
        PoseVelocity2d vel;
        if(gamepad1.back && !rt1.isDown()) vel = new PoseVelocity2d(new Vector2d(0, 0), 0);
        else vel = new PoseVelocity2d(new Vector2d(speed * -gamepad1.left_stick_y, speed * -gamepad1.left_stick_x), speed * -gamepad1.right_stick_x);
        robot.drive.setDrivePowers(vel);
        robot.drive.updatePoseEstimate();

        // intake
        if(rt2.wasJustPressed() || (gamepad1.back && rt1.wasJustPressed())) {
            runningActions.add(robot.prepareForIntake());
//            runningActions.add(robot.intake.inWithPeriodicOut());
        }
        double lt = driver2.getTrigger(Trigger.LEFT_TRIGGER);
        if(lt > 0)
            robot.intake.out(lt);
        else if(rt2.isDown()) {
            robot.outtake.flipper.goToMinPos();
            robot.intake.in();
        } else {
            robot.intake.stop();
//            cancel(Intake.InWithPeriodicOut.class);
        }

        // outtake
        if(gamepad2.back) {
            if (gamepad2.dpad_up)
                robot.outtake.flipper.rotateIncrementally();
            else if (gamepad2.dpad_down)
                robot.outtake.flipper.unrotateIncrementally();
            robot.outtake.flipper.rotateBy(-gamepad2.left_stick_y / 300);
        } else {
            if (driver2.wasJustPressed(Button.DPAD_UP)) {
                if(Objects.equals(robot.outtake.flipper.getState(), Flipper.UP))
                    robot.outtake.extender.goToMaxPos();
                else {
                    robot.outtake.releaser.close();
                    cancel(robot.outtake.getLowerActionClass());
                    runningActions.add(robot.outtake.raise());
                }
            } else if (driver2.wasJustPressed(Button.DPAD_DOWN))
                runningActions.add(robot.outtake.lower());
            if(!gamepad1.back || !rt1.isDown())
                robot.outtake.motor.setPower(driver2.getLeftY());
            else robot.outtake.motor.stop();
            robot.outtake.extender.rotateBy(-gamepad2.right_stick_y / 100);
        }
        if(!gamepad1.back || !rt1.isDown()) {
            robot.outtake.armRotator.rotateBy(driver2.getLeftX() / 200);
            robot.outtake.pixelRotator.rotateBy(driver2.getRightX() / 100);
        }
        if(driver2.wasJustPressed(Button.LEFT_STICK_BUTTON)) {
            if(driver2.isDown(Button.START)) robot.outtake.armRotator.setCenterPos();
            else robot.outtake.armRotator.center();
        }
        if(driver2.wasJustPressed(Button.RIGHT_STICK_BUTTON)) {
            if(driver2.isDown(Button.START)) robot.outtake.pixelRotator.setCenterPos();
            else robot.outtake.pixelRotator.center();
        }
        if(driver2.isDown(Button.DPAD_RIGHT)) robot.outtake.moveRight();
        else if(driver2.isDown(Button.DPAD_LEFT)) robot.outtake.moveLeft();
        if(driver2.isDown(Button.A) && !driver2.isDown(Button.START)) robot.outtake.releaser.open();
        else if(driver2.isDown(Button.B) && !driver2.isDown(Button.START)) robot.outtake.releaser.close();

        // hang
        if(driver2.isDown(Button.BACK))
            robot.hang.setPower(-driver2.getRightY());

        // launcher
//        if(driver2.isDown(Button.RIGHT_BUMPER)) robot.launcher.rotate();
//        else if(driver2.isDown(Button.LEFT_BUMPER)) robot.launcher.unrotate();

        // auto claw
        if(driver2.isDown(Button.X))
            robot.autoClaw.down(.01);
        else if(driver2.isDown(Button.Y))
            robot.autoClaw.up(.01);

        // clear running actions
        if((gamepad1.start && gamepad1.x) || (gamepad2.start && gamepad2.x))
            runningActions.clear();
    }

    private boolean currentlyRunning(Class targetAction) {
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
    private void cancel(Class actionToCancel) {
        if(actionToCancel == null) return;
        for(int i = 0; i < runningActions.size(); i++) {
            if(runningActions.get(i).getClass().equals(actionToCancel))
                runningActions.remove(i);
        }
    }
}
