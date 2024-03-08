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
import org.firstinspires.ftc.teamcode.apriltags.AprilTagDetector;
import org.firstinspires.ftc.teamcode.apriltags.AprilTagIDs;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.Location;
import org.firstinspires.ftc.teamcode.enums.Side;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.openftc.apriltag.AprilTagDetection;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Autonomous(preselectTeleOp = "Tele")
public class Auto extends LinearOpMode {
    private enum ParkPosition {
        CORNER, CENTER
    }
    private static Alliance alliance = Alliance.BLUE;
    private static Side side = Side.FAR;
    private static boolean wait = false, goThroughStageDoor = true, placeOnBackdrop = true, useAprilTags = true, pickFromStack = true;
    private static ParkPosition parkPosition = ParkPosition.CENTER;
    private static boolean debugMode = false;

    private Robot robot;
    private TensorFlowObjectDetector propDetector;
    private AprilTagDetector aprilTagDetector;
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
        public Vector2d backdropPose = newVector(60, 35), stackPose, parkPose;
        public final Vector2d aprilTagOffset = new Vector2d(-15, 0);
        public Vector2d trussFront, trussBack;
        public int reversed = (alliance == Alliance.BLUE) ? 1 : -1;
        public Trajectories() {
            startPose = (side == Side.NEAR) ? newPose(17, 64, Math.PI/2) : newPose(-42+3, 64, Math.PI/2);
            trussFront = newVector(-30, goThroughStageDoor ? -1 : 63);
            trussBack = new Vector2d(33, trussFront.y);
            stackPose = newVector(-57, 1); // <- stack closest to center. middle stack coords: (-57, 20)
            if(side == Side.FAR) stackPose = stackPose.plus(new Vector2d(-5+2, 0));
//            if(side == Side.FAR) backdropPose = backdropPose.plus(new Vector2d(-4, 6));
            parkPose = (parkPosition == ParkPosition.CORNER) ? newVector(66-10, 55) : newVector(66-10, 8);
        }
        public Action generateSpikeMarkTraj() {
            if(side == Side.NEAR) {
                if ((alliance == Alliance.BLUE && propLocation == Location.LEFT) || (alliance == Alliance.RED && propLocation == Location.RIGHT)) {
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToSplineHeading(newVector(35, 30), Math.PI)
                            .build();
                } else if (propLocation == Location.CENTER) {
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToSplineHeading(newVector(24+2, 17), Math.PI)
                            .build();
                } else { // blue right/red left
                    Vector2d spikeMarkPose = newVector(10, 25+2);
                    double backAmount = 3;
                    if(robot.drive.pose.position.x > 40)
                        return robot.drive.actionBuilder(robot.drive.pose)
                                .turnTo(Math.PI)
                                .strafeTo(spikeMarkPose)
                                .lineToX(spikeMarkPose.x + backAmount)
                                .build();
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeTo(newVector(12, 40))
                            .turnTo(Math.PI)
                            .strafeTo(spikeMarkPose)
                            .lineToX(spikeMarkPose.x + backAmount)
                            .build();
                }
            }
            else if(side == Side.FAR) {
                if ((alliance == Alliance.BLUE && propLocation == Location.LEFT) || (alliance == Alliance.RED && propLocation == Location.RIGHT)) {
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToSplineHeading(newVector(-35, 40), 0)
//                            .turnTo(0)
                            .strafeTo(newVector(-30, 29))
                            .build();
                } else if (propLocation == Location.CENTER) {
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToSplineHeading(newVector(-57+3, 17), Math.PI/6 * reversed)
//                            .turnTo(Math.toRadians(30))
                            .build();
                } else { // blue right/red left
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeTo(newVector(-33, 18))
                            .strafeTo(newVector(-51, 16))
                            .build();
                }
            }
            return null;
        }

        public Action generateBackdropTraj(boolean careAboutPosition, Action armMovement) {
            if(!careAboutPosition) {
                double dist = Math.abs(robot.drive.pose.position.x);
                return robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToSplineHeading(newVector(robot.drive.pose.position.x+5, -2), Math.PI)
//                        .turnTo(Math.PI)
                        .strafeTo(newVector(backdropPose.x, 18))
                        .afterDisp(dist, robot.outtake.raise())
                        .afterDisp(dist + 40, robot.outtake.extender::goToMaxPos)
                        .afterDisp(dist + 50, robot.outtake.goToLeft())
                        .build();
            }
            Vector2d newBackdropPose = backdropPose;
            if(robot.drive.pose.position.x > 0)
                return robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToSplineHeading(newBackdropPose, Math.PI)
                        .afterDisp(1, new SequentialAction(robot.outtake.extender.goToMinPosWithActions(), robot.outtake.raise(), armMovement))
                        .build();
            else if(pickFromStack)
                return robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToSplineHeading(newVector(robot.drive.pose.position.x, trussBack.y), Math.PI)
//                        .turnTo(Math.PI)
                        .strafeTo(trussBack)
                        .strafeTo(newBackdropPose)
                        .afterDisp(1, new SequentialAction(robot.outtake.raise(), armMovement))
                        .build();
            else
                return robot.drive.actionBuilder(robot.drive.pose)
                        .strafeTo(trussFront)
                        .turnTo(Math.PI)
                        .strafeTo(trussBack)
                        .afterDisp(50, robot.outtake::raise)
                        .strafeTo(newBackdropPose)
                        .build();
        }
        public Action generateToAprilTagDetection() {
            return robot.drive.actionBuilder(robot.drive.pose)
                .strafeToSplineHeading(backdropPose.plus(aprilTagOffset), Math.PI)
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
//                    .turnTo(Math.PI)
                    .strafeToSplineHeading(stackPose, Math.PI)
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
                    .strafeTo(parkPose)
                    .afterTime(.2, robot.outtake.lower())
//                    .lineToX(65)
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

        try {
            aprilTagDetector = new AprilTagDetector(hardwareMap);
        } catch (Exception e) {
            telemetry.log().add("opencv init failed \n" + e);
        }

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
                if(useAprilTags)
                    updatePoseFromAprilTag();
                Actions.runBlocking(new ParallelAction(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeTo(trajectories.backdropPose)
                                .build(),
                        new SequentialAction(
                                robot.outtake.raise(),
                                getArmMovementAction()
                        )
                ));
                placeOnBackdrop();
            } else {
                if(useAprilTags) {
                    Actions.runBlocking(new ParallelAction(
                            trajectories.generateToAprilTagDetection(),
                            robot.outtake.raise()
                    ));
                    updatePoseFromAprilTag();
                }
                Actions.runBlocking(new ParallelAction(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeTo(trajectories.backdropPose)
                                .build(),
                        getArmMovementAction()
                ));
                placeOnBackdrop();
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
                Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToSplineHeading(newVector(robot.drive.pose.position.x, trajectories.trussBack.y), Math.PI)
                        .strafeTo(trajectories.trussBack).build());
            } else if(wait) {
                Actions.runBlocking(new SequentialAction(
                        robot.drive.actionBuilder(robot.drive.pose)
                            .strafeTo(trajectories.trussFront)
                            .build(),
                        new SleepAction(getRuntime() - WAIT_TIME),
                        robot.drive.actionBuilder(robot.drive.pose)
                            .strafeTo(trajectories.trussBack)
                            .build()
                ));
            }
            if(useAprilTags) {
                Actions.runBlocking(new ParallelAction(
                        trajectories.generateToAprilTagDetection(),
                        robot.outtake.raise()
                ));
                updatePoseFromAprilTag();
            }
            moveToBackdrop(true);
            placeOnBackdrop();
        }
        if(debugMode) pause();

        if(pickFromStack) {
            pickFromStack();
//            if(debugMode) pause();
            moveToBackdrop(false);
//            if(debugMode) pause();
            placeOnBackdrop();
        }

        status.setValue("parking at %.1fs", getRuntime());
        telemetry.update();
        Actions.runBlocking(trajectories.generateParkTraj());

        status.setValue("parked in %.1f s", getRuntime());
        telemetry.update();
        Tele.setStartPose(robot.drive.pose);

        Actions.runBlocking(new SleepAction(3+2));
        if(debugMode) {
            telemetry.log().add("press to reset");
            pause();
            Actions.runBlocking(trajectories.generateResetTraj());
        }
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
//        robot.intake.out(.5);
//        executorService.schedule(robot.intake::stop, 250, TimeUnit.MILLISECONDS);
    }

    private void moveToBackdrop(boolean careAboutPosition) {
        Action armMovement = careAboutPosition ? getArmMovementAction() : new SequentialAction(
                new InstantAction(robot.outtake.extender::goToMaxPos),
                new SleepAction(.75),
                robot.outtake.goToLeft()
        );
        Actions.runBlocking(trajectories.generateBackdropTraj(careAboutPosition, armMovement));
    }
    private void placeOnBackdrop() {
        Actions.runBlocking(new SequentialAction(
                new SleepAction(1-.2),
                new InstantAction(robot.outtake.releaser::open),
                new SleepAction(1-.5)
        ));
    }
    private Action getArmMovementAction() {
        switch (propLocation) {
            case LEFT:
                return robot.outtake.goToLeft();
            case RIGHT:
                return robot.outtake.goToRight();
            case CENTER:
                if(side == Side.FAR)
                    return robot.outtake.extender.goToMaxPosWithActions();
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
//        if(debugMode) pause();
        robot.intake.in();
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)
                .strafeTo(robot.drive.pose.position.plus(newVector(-3, 10))) //, new TranslationalVelConstraint(10*2))
                .build()
        );
//        robot.intake.stop();
        robot.intake.out();
        executorService.schedule(robot.intake::stop, 1_500, TimeUnit.MILLISECONDS);
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
            telemetry.log().add("tensorflow init failed \n" + e);
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
        telemetry.addData("use april tags (LB)", () -> placeOnBackdrop ? useAprilTags : "n/a");
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
    private static String poseToString(Pose2d pose) {
        return String.format("(%.1f, %.1f) @ %.1f deg", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
    }
    private void updatePoseFromAprilTag() {
        final double MAX_DETECTION_TIME = 1.0, MAX_CHANGE = 10;
        status.setValue("looking for april tags...");
        telemetry.update();
        timer.reset();
        AprilTagDetection detectedTag = null;
        while(detectedTag == null && timer.seconds() < MAX_DETECTION_TIME) {
            detectedTag = aprilTagDetector.detect();
        }
        if(detectedTag == null) {
            telemetry.log().add("didn't detect anything in %.1fs", MAX_DETECTION_TIME);
            return;
        }
        Object[] location = AprilTagIDs.getLocation(detectedTag.id);
        if(location == null) {
            telemetry.log().add("tag not recognized (id %d)", detectedTag.id);
            return;
        }
        telemetry.log().add("detected %s %s after %.2fs", location[0], location[1], timer.seconds());
        Vector2d cameraTagPose = AprilTagDetector.convertPose(detectedTag.pose);
        telemetry.log().add("april tag pose (camera): %s", vectorToString(cameraTagPose));
        Vector2d fieldTagPose = aprilTagDetector.aprilTagIDs.getPose(detectedTag.id);
        telemetry.log().add("april tag pose (field): %s", vectorToString(fieldTagPose));
        Pose2d newRobotPose = newPose(fieldTagPose.x - cameraTagPose.x, fieldTagPose.y + cameraTagPose.y, robot.drive.pose.heading.toDouble());
        telemetry.log().add("old robot pose: %s", vectorToString(robot.drive.pose.position));
        telemetry.log().add("new robot pose: %s", vectorToString(newRobotPose.position));
        Vector2d change = newRobotPose.position.minus(robot.drive.pose.position);
        telemetry.log().add("change: %s", vectorToString(change));
        telemetry.update();
        if(Math.abs(change.x) < MAX_CHANGE && Math.abs(change.y) < MAX_CHANGE)
            robot.drive.pose = newRobotPose;
        else telemetry.log().add("did not update (change too big)");
//        if(debugMode) pause();
    }
    private static String vectorToString(Vector2d v) {
        return String.format("(%.2f, %.2f)", v.x, v.y);
    }

    private Vector2d newVector(double x, double y) {
        if(alliance == Alliance.BLUE)
            return new Vector2d(x, y);
        else
            return new Vector2d(x, -y);
    }
    private Vector2d newVector(Vector2d v) {
        if(alliance == Alliance.BLUE)
            return v;
        else
            return new Vector2d(v.x, -v.y);
    }
    private Pose2d newPose(Pose2d pose) {
        if(alliance == Alliance.BLUE)
            return pose;
        else
            return new Pose2d(newVector(pose.position), pose.heading.inverse().toDouble());
    }
    private Pose2d newPose(double x, double y, double heading) {
        return newPose(new Pose2d(x, y, heading));
    }
}
