package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.Location;
import org.firstinspires.ftc.teamcode.enums.Side;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Autonomous//(preselectTeleOp = "Tele")
public class Auto extends LinearOpMode {
    private enum ParkPosition {
        CORNER, CENTER
    }
    private static Alliance alliance = Alliance.BLUE;
    private static Side side = Side.FAR;
    private static boolean wait = false, goThroughStageDoor = true, placeOnBackdrop = true, useAprilTags = false, pickFromStack = true;
    private static ParkPosition parkPosition = ParkPosition.CENTER;
    private static boolean debugMode = true;

    private Robot robot;
    private TensorFlowObjectDetector propDetector;
    private Location propLocation;
    private ElapsedTime timer;
    private ScheduledExecutorService executorService;
    private boolean initialized = false, propLocationOverride = false;
    private static final double MIN_INIT_TIME = 5, WAIT_TIME = 15;
    private double beginningWaitTime = 0;
    private Telemetry.Item status, propLocationTelemetry;
    private GamepadEx driver;
    private TriggerReader rt, lt;
    private CustomButton back;
    private class Trajectories {
        public Pose2d startPose;
        public Vector2d backdropPose = new Vector2d(60, 35), stackPose, parkPose;
        public final Vector2d aprilTagOffset = new Vector2d(-7, 0);
        public Vector2d trussFront, trussBack;
        public Trajectories() {
            startPose = (side == Side.NEAR) ? new Pose2d(17, 64, Math.PI/2) : new Pose2d(-42+3, 64, Math.PI/2);
            trussFront = new Vector2d(-30, goThroughStageDoor ? 1 : 63);
            trussBack = new Vector2d(30+3, trussFront.y);
            stackPose = new Vector2d(-57, 1); // <- stack closest to center. middle stack coords: (-57, 20)
            if(side == Side.FAR) backdropPose = backdropPose.plus(new Vector2d(-4, 6));
            parkPose = (parkPosition == ParkPosition.CORNER) ? new Vector2d(66, 55) : new Vector2d(66, 8);
        }
        public Action generateSpikeMarkTraj() {
            if(side == Side.NEAR) {
                if ((alliance == Alliance.BLUE && propLocation == Location.LEFT) || (alliance == Alliance.RED && propLocation == Location.RIGHT)) {
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToSplineHeading(new Vector2d(35, 30), Math.PI)
                            .build();
                } else if (propLocation == Location.CENTER) {
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToSplineHeading(new Vector2d(22+2, 17), Math.PI)
                            .build();
                } else { // blue right/red left
                    Vector2d spikeMarkPose = new Vector2d(10, 25+2);
                    double backAmount = 3;
                    if(robot.drive.pose.position.x > 40)
                        return robot.drive.actionBuilder(robot.drive.pose)
                                .turnTo(Math.PI)
                                .strafeTo(spikeMarkPose)
                                .lineToX(spikeMarkPose.x + backAmount)
                                .build();
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeTo(new Vector2d(12, 40))
                            .turnTo(Math.PI)
                            .strafeTo(spikeMarkPose)
                            .lineToX(spikeMarkPose.x + backAmount)
                            .build();
                }
            }
            else if(side == Side.FAR) {
                if ((alliance == Alliance.BLUE && propLocation == Location.LEFT) || (alliance == Alliance.RED && propLocation == Location.RIGHT)) {
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeTo(new Vector2d(-35, 40))
                            .turnTo(0)
                            .strafeTo(new Vector2d(-30, 28 + 1))
                            .build();
                } else if (propLocation == Location.CENTER) {
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeTo(new Vector2d(-49 - 2, 17))
                            .turnTo(Math.toRadians(35))
                            .build();
                } else { // blue right/red left
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeTo(new Vector2d(-34+1, 18))
                            .strafeTo(new Vector2d(-51, 18-2))
                            .build();
                }
            }
            return null;
        }

        public Action generateBackdropTraj(boolean careAboutPosition, Action armMovement) {
            if(!careAboutPosition)
                return robot.drive.actionBuilder(robot.drive.pose)
                        .strafeTo(new Vector2d(backdropPose.x, 15+3))
                        .afterDisp(Math.abs(trussBack.x - robot.drive.pose.position.x), new SequentialAction(robot.outtake.raise(), armMovement))
                        .build();
            if(robot.drive.pose.position.x > 0)
                return robot.drive.actionBuilder(robot.drive.pose)
//                    .setReversed(true)
                        .strafeToSplineHeading(backdropPose, Math.PI)
                        .build();
            else if(pickFromStack)
                return robot.drive.actionBuilder(robot.drive.pose)
                        .strafeTo(trussBack)
                        .strafeTo(backdropPose)
                        .afterDisp(1, new SequentialAction(robot.outtake.raise(), armMovement))
                        .build();
            else
                return robot.drive.actionBuilder(robot.drive.pose)
                        .strafeTo(trussFront)
                        .turnTo(Math.PI)
                        .strafeTo(trussBack)
                        .afterDisp(50, robot.outtake::raise)
                        .strafeTo(backdropPose)
                        .build();
        }
        public Action generateToStack() {
            if(robot.drive.pose.position.x > 0)
                return robot.drive.actionBuilder(robot.drive.pose)
                        .strafeTo(trussBack)
    //                    .strafeTo(trussFront)
                        .strafeTo(stackPose)
                        .build();
            else return robot.drive.actionBuilder(robot.drive.pose)
                    .turnTo(Math.PI)
                    .strafeTo(stackPose)
                    .build();
        }
        public Action generateParkTraj() {
            if(pickFromStack || wait)
                return new ParallelAction(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .lineToX(robot.drive.pose.position.x - 2)
                                .build(),
                        robot.outtake.lower()
                );
            return robot.drive.actionBuilder(robot.drive.pose)
                    .lineToX(robot.drive.pose.position.x - 5)
//                    .setReversed(true)
                    .strafeTo(new Vector2d(robot.drive.pose.position.x, parkPose.y))
                    .afterTime(.2, robot.outtake.lower())
                    .lineToX(parkPose.x)
                    .build();
        }
        public Action generateResetTraj() {
            return robot.drive.actionBuilder(robot.drive.pose)
                    .strafeToSplineHeading(startPose.position, startPose.heading.toDouble())
                    .build();
        }
    }
    private Trajectories trajectories;

    @Override
    public void runOpMode() {
        initialize();
        initLoop();
        double initTime = getRuntime();
        resetRuntime();
        telemetry.clearAll();
        if(!propLocationOverride && initTime < MIN_INIT_TIME) {
            beginningWaitTime = MIN_INIT_TIME - initTime;
            continueDetectingTeamProp(beginningWaitTime);
        } else if(side == Side.NEAR && wait) {
            beginningWaitTime = WAIT_TIME;
            waitWithTelemetry();
        }
        try {
            propDetector.stopDetecting();
        } catch (Exception ignored) {}

        telemetry.clearAll();
        runtimeTelemetry();
        telemetry.update();

        trajectories = new Trajectories();
        robot.initDrive(hardwareMap, trajectories.startPose);
        telemetry.addData("pos", () -> poseToString(robot.drive.pose));

//        robot.autoClaw.setPosition(.6);
        robot.outtake.releaser.close();

        if(side == Side.NEAR) {
            if (!pickFromStack || (alliance == Alliance.BLUE && propLocation == Location.LEFT) || (alliance == Alliance.RED && propLocation == Location.RIGHT)) {
                placePurplePixel();
                placeOnBackdrop(true);
            } else {
                placeOnBackdrop(true);
                if(debugMode) pause();
                placePurplePixel();
                if (propLocation == Location.RIGHT)
                    Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)
                            .lineToX(trajectories.startPose.position.x)
                            .build()
                    );
            }
        } else if(side == Side.FAR) {
            placePurplePixel();
            if(debugMode) pause();
            if(pickFromStack) {
                pickFromStack();
                if(debugMode) pause();
            } else if(wait)
                wait(getRuntime() - WAIT_TIME);
            placeOnBackdrop(true);
        }
        if(debugMode) pause();

        if(pickFromStack) {
            pickFromStack();
            if(debugMode) pause();
            placeOnBackdrop(false);
            if(debugMode) pause();
        }

        status.setValue("parking at %.1fs", getRuntime());
        telemetry.update();
        Actions.runBlocking(trajectories.generateParkTraj());

        status.setValue("parked in %.1f s", getRuntime());
        telemetry.update();
        if(debugMode) {
            telemetry.log().add("press to reset");
            pause();
            Actions.runBlocking(trajectories.generateResetTraj());
        } else {
            timer.reset();
            while (timer.seconds() < 3 && opModeIsActive()) ;
        }

        Tele.setStartPose(robot.drive.pose);
    }

    private void runtimeTelemetry() {
        telemetry.setAutoClear(false);
        telemetry.addData("Running", "%s %s,  prop location = %s", alliance, side, propLocation);
        if(wait)
            telemetry.addLine("WAIT");
        if(side == Side.FAR || pickFromStack)
            telemetry.addLine("Go through " + (goThroughStageDoor ? "stage door" : "truss by wall"));
        telemetry.addLine((placeOnBackdrop ? "Place" : "Don't place") + " on backdrop");
        if(pickFromStack)
            telemetry.addLine("Pick from stack");
        if(placeOnBackdrop)
            telemetry.addLine((useAprilTags ? "Use" : "Don't use") + " april tags");
        if((!wait && !pickFromStack) || !placeOnBackdrop)
            telemetry.addData("Park in", parkPosition);
        if(debugMode)
            telemetry.addLine("DEBUG");
        status = telemetry.addData("\nStatus", "loading...");
        telemetry.addData("runtime", "%.1f", this::getRuntime);
    }

    private void placePurplePixel() {
        Actions.runBlocking(new ParallelAction(
                trajectories.generateSpikeMarkTraj(),
                robot.outtake.lower()
        ));
//        robot.autoClaw.setPosition(.5);
        robot.intake.out(.5);
        executorService.schedule(robot.intake::stop, 250, TimeUnit.MILLISECONDS);
    }

    private void placeOnBackdrop(boolean careAboutPosition) {
        Action armMovement = careAboutPosition ? getArmMovementAction() : new ParallelAction(
                new InstantAction(robot.outtake.extender::goToMaxPos), robot.outtake.goToLeft()
        );
        Actions.runBlocking(new SequentialAction(
                trajectories.generateBackdropTraj(careAboutPosition, armMovement),
                new SleepAction(.5),
                new InstantAction(robot.outtake.releaser::open),
                new SleepAction(.5)
        ));
    }
    private Action getArmMovementAction() {
        switch (propLocation) {
            case LEFT:
                return robot.outtake.goToLeft();
            case RIGHT:
                return robot.outtake.goToRight();
            default:
                return new NullAction();
        }
    }
    private void pickFromStack() {
        status.setValue("moving to stack");
        telemetry.update();
        Actions.runBlocking(new ParallelAction(
                trajectories.generateToStack(),
                robot.prepareForIntake()
        ));
        if(debugMode) pause();
        robot.intake.in();
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)
//                .lineToX(robot.drive.pose.position.x - 2, new TranslationalVelConstraint(2))
                .strafeTo(robot.drive.pose.position.plus(new Vector2d(-1, 5)), new TranslationalVelConstraint(5))
                .build()
        );
        robot.intake.stop();
//        robot.intake.out();
//        executorService.schedule(robot.intake::stop, 1, TimeUnit.SECONDS);
    }

    private void wait(double time) {
        timer.reset();
        while(timer.seconds() < time && opModeIsActive());
    }

    private void pause() {
        status.setValue("paused (press a)");
        telemetry.update();
        while(!gamepad2.a && opModeIsActive());
        status.setValue("resuming");
        telemetry.update();
    }

    private void initialize() {
        status = telemetry.addData("Status", "initializing...").setRetained(true);
        telemetry.update();

        robot = new Robot(hardwareMap, telemetry);
        robot.outtake.releaser.open();
        try {
            propDetector = new TensorFlowObjectDetector(hardwareMap);
        } catch(Exception e) {
            telemetry.log().add("camera init failed: " + e);
        }
        propLocation = Location.LEFT;
        timer = new ElapsedTime();
        executorService = Executors.newSingleThreadScheduledExecutor();

        driver = new GamepadEx(gamepad2);
        rt = new TriggerReader(driver, GamepadKeys.Trigger.RIGHT_TRIGGER);
        lt = new TriggerReader(driver, GamepadKeys.Trigger.LEFT_TRIGGER);
        back = new CustomButton(driver, Button.BACK);

        telemetry.addData("debug mode (start + x/y)", () -> debugMode);
        telemetry.addLine().addData("alliance", () -> alliance).addData("side", () -> side);
        telemetry.addData("wait (RB)", () -> pickFromStack ? "n/a" : wait);
        telemetry.addData("go through stage door (LS)", () -> (side == Side.FAR || pickFromStack) ? goThroughStageDoor : "n/a");
        telemetry.addData("place on backdrop (LT)", () -> placeOnBackdrop);
        telemetry.addData("pick from stack (RT)", () -> pickFromStack);
//        telemetry.addData("use april tags (LB)", () -> placeOnBackdrop ? useAprilTags : "n/a");
        telemetry.addData("park (RS)", () -> placeOnBackdrop && (wait || pickFromStack) ? "n/a" : parkPosition);
        propLocationTelemetry = telemetry.addData("prop location", null).setRetained(true);
    }
    private void initLoop() {
        while(opModeInInit()) {
            if(!initialized) {
                if(getRuntime() < MIN_INIT_TIME)
                    status.setValue("initializing...%.1f", MIN_INIT_TIME - getRuntime());
                else {
                    initialized = true;
                    status.setValue("initialized");
                }
            }

            getGamepadInput();

            if (!propLocationOverride) {
                try {
                    detectTeamProp();
                } catch (Exception e) {
                    propLocationTelemetry.setValue("error: " + e);
                }
            }

            telemetry.update();
        }
    }
    private void getGamepadInput() {
        driver.readButtons();
        rt.readValue();
        lt.readValue();
        back.update();

        if((gamepad1.start && gamepad1.back) || (gamepad2.start && gamepad2.back))
            requestOpModeStop();

        // ALLIANCE
        if(gamepad2.x && !gamepad2.start) alliance = Alliance.BLUE;
        else if(gamepad2.b && !gamepad2.start) alliance = Alliance.RED;

        // SIDE
        if(gamepad2.a && !gamepad2.start) {
            side = Side.NEAR;
            wait = false;
            parkPosition = ParkPosition.CORNER;
        } else if(gamepad2.y && !gamepad2.start) {
            side = Side.FAR;
            wait = true;
        }

        // WAIT
        if(driver.wasJustReleased(Button.RIGHT_BUMPER) && !gamepad2.back) wait = !wait;

        // GO THROUGH STAGE DOOR
        if(!gamepad2.back) {
            if (gamepad2.left_stick_y < -.5) goThroughStageDoor = true;
            else if (gamepad2.left_stick_y > .5) goThroughStageDoor = false;
        }

        // PLACE ON BACKDROP
        if (lt.wasJustReleased() && !gamepad2.back) placeOnBackdrop = !placeOnBackdrop;

        // PICK FROM STACK
        if(rt.wasJustPressed()) {
            pickFromStack = !pickFromStack;
            if(pickFromStack) {
                wait = false;
                goThroughStageDoor = true;
            } else if(side == Side.FAR) wait = true;
        }

        // APRIL TAGS
        if(placeOnBackdrop && driver.wasJustReleased(Button.LEFT_BUMPER)) useAprilTags = !useAprilTags;

        // PARK
        if(!gamepad2.back) {
            if (gamepad2.right_stick_y < -.1)
                parkPosition = ParkPosition.CENTER;
            else if (gamepad2.right_stick_y > .1)
                parkPosition = ParkPosition.CORNER;
        }

        // DEBUG MODE
        if(gamepad2.start && gamepad2.x) debugMode = true;
        else if(gamepad2.start && gamepad2.y) debugMode = false;

        // PROP LOCATION OVERRIDE
        if (gamepad2.dpad_left || gamepad2.dpad_up || gamepad2.dpad_right) {
            propLocationOverride = true;
            if (gamepad2.dpad_left)
                propLocation = Location.LEFT;
            else if (gamepad2.dpad_up)
                propLocation = Location.CENTER;
            else
                propLocation = Location.RIGHT;
            propLocationTelemetry.setValue(propLocation + " (override)");
        } else if (back.getState() == CustomButton.State.JUST_UP && back.getTimeDown() < .5)
            propLocationOverride = false;

        // PRELOAD
        if(gamepad2.back) {
            if (gamepad2.a && !gamepad2.start) robot.outtake.releaser.open();
            robot.autoClaw.rotateBy((gamepad2.left_stick_y + gamepad2.right_stick_y) / 300);
        }
    }
    private void detectTeamProp() {
        propDetector.update();
        propLocation = propDetector.getLocation();
        propLocationTelemetry.setValue(propLocation);
        telemetry.addLine();
        propDetector.telemetryAll(telemetry);
    }
    private void continueDetectingTeamProp(double time) {
        status.setValue("detecting prop location...%.1f", () -> time - getRuntime());
        propLocationTelemetry = telemetry.addData("prop location", propLocation);
        while(getRuntime() < time && opModeIsActive()) {
            detectTeamProp();
            telemetry.update();
        }
    }
    private void waitWithTelemetry() {
        status.setValue("waiting...%.1f", () -> WAIT_TIME - getRuntime());
        while(getRuntime() < WAIT_TIME && opModeIsActive())
            telemetry.update();
    }
    public static Alliance getAlliance() { return alliance; }
    public static Side getSide() {
        return side;
    }
    private static String poseToString(Pose2d startPose) {
        return String.format("(%.1f, %.1f) @ %.1f deg", startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble()));
    }
}
