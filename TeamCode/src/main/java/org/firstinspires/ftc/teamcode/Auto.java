package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.util.Timing;
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

@Autonomous(preselectTeleOp = "TeleOp")
public class Auto extends LinearOpMode {
    private enum ParkPosition {
        CORNER, CENTER
    }
    private static Alliance alliance = Alliance.BLUE;
    private static Side side = Side.FAR;
    private static boolean wait = false, goThroughStageDoor = true, placeOnBackdrop = true, useAprilTags = true, pickFromStack = true;
    private static ParkPosition parkPosition = ParkPosition.CORNER;
    private static boolean debugMode = false;

    private Robot robot;
    private TensorFlowObjectDetector propDetector;
    private AprilTagDetector aprilTagDetector;
    private Location propLocation;
    private Timing.Timer runTimer;
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
        public Vector2d backdropPose, stackPose, parkPose;
        public final Vector2d aprilTagOffset = new Vector2d(alliance == Alliance.BLUE ? -17 : -17-5, 0);
        public Vector2d trussFront, trussBack;
        public int reversed = (alliance == Alliance.BLUE) ? 1 : -1;
        public Trajectories() {
            double startX;
            if(side == Side.NEAR) startX = (alliance == Alliance.BLUE) ? 1 : 13+1;
            else startX = alliance == Alliance.BLUE ? -42+3 : -25;
            startPose = newPose(startX, 64, -Math.PI/2);

            stackPose = newVector(-58, -3); // <- stack closest to center. middle stack coords: (-57, 20)
            if(side == Side.FAR) stackPose = stackPose.plus(new Vector2d(-2-2, -8-5));
//            if(blue() && !near() && !left()) stackPose = stackPose.plus(new Vector2d(-3, 3));

            trussFront = newVector(-35, goThroughStageDoor ? 5 : 63);
            if(side == Side.FAR) trussFront = trussFront.plus(new Vector2d(0, blue() ? -4 : -4-5));
            else if(alliance == Alliance.RED) trussFront = trussFront.plus(new Vector2d(0, 6+2));
            trussBack = new Vector2d(33, trussFront.y + (blue() && !near() ? -15-5 : 5+3));

            backdropPose = new Vector2d(61, alliance == Alliance.BLUE ? 30-.3 : -31+4);
            if(side == Side.FAR) backdropPose = backdropPose.plus(newVector(blue() ? -3+1 : -4, goThroughStageDoor ? 5 : -5));

            parkPose = newVector(56, parkPosition == ParkPosition.CORNER ? 69+1 : -9);
            if(alliance == Alliance.RED && side == Side.FAR) parkPose = parkPose.plus(new Vector2d(0, -2));
        }
        public Action generateSpikeMarkTraj() {
            if(side == Side.NEAR) {
                if ((alliance == Alliance.BLUE && propLocation == Location.LEFT) || (alliance == Alliance.RED && propLocation == Location.RIGHT)) {
                    Vector2d pose = alliance == Alliance.BLUE ? new Vector2d(28-.5, 40) : new Vector2d(25-.5, -40.5+1);
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToSplineHeading(pose, -Math.PI/2 * reversed)
                            .build();
                } else if (propLocation == Location.CENTER) {
                    if(pickFromStack && blue())
                        return robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToSplineHeading(new Vector2d(26, 13-.5), Math.PI)
                                .build();
                    Vector2d pose = alliance == Alliance.BLUE ? new Vector2d(14, 30-1) : new Vector2d(23.5, -12-2);
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToSplineHeading(pose, alliance == Alliance.BLUE ? -Math.PI/2 : Math.PI)
                            .build();
                } else { // blue right/red left
                    Vector2d spikeMarkPose = newVector(alliance == Alliance.BLUE ? 8.0+1 : 7+5, 32);
                    if(robot.drive.pose.position.x > 40)
                        return robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToSplineHeading(spikeMarkPose, Math.toRadians(-170 * reversed))
                                .build();
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToSplineHeading(newVector(16, 40), Math.toRadians(-160 * reversed))
                            .strafeTo(spikeMarkPose)
                            .build();
                }
            }
            else if(side == Side.FAR) {
                if ((alliance == Alliance.BLUE && propLocation == Location.LEFT) || (alliance == Alliance.RED && propLocation == Location.RIGHT)) {
                    return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToSplineHeading(newVector(-38, 36), 0)
//                            .turnTo(0)
                            .strafeTo(blue() ? new Vector2d(-27.5-1.5, 29) : new Vector2d(-31, -28+6))
                            .build();
                } else if (propLocation == Location.CENTER) {
                    if(alliance == Alliance.BLUE)
                        return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToSplineHeading(newVector(-52.5, 13), 0)
                            .build();
                    else return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToSplineHeading(new Vector2d(-56, -6+1), 0)
                            .build();
                } else { // blue right/red left
                    if(goThroughStageDoor || pickFromStack || alliance == Alliance.RED)
                        return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeTo(newVector(blue() ? -33 : -34, 18))
                            .strafeToSplineHeading(newVector(blue() ? -52-.5 : -43, 16), alliance == Alliance.BLUE ? Math.PI/2 : -Math.PI/2)
                            .build();
                    else return robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToSplineHeading(new Vector2d(-45-2, 36+2), -Math.PI/2)
                            .build();
                }
            }
            return null;
        }

        public Action generateBackdropTraj(boolean careAboutPosition, Action armMovement) {
            if(!careAboutPosition) {
                return robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToSplineHeading(newVector(robot.drive.pose.position.x+5, trajectories.trussFront.y), Math.PI)
//                        .turnTo(Math.PI)
                        .strafeTo(new Vector2d(5, trussBack.y))
                        .strafeToSplineHeading(newVector(backdropPose.x-3, propLocation == Location.RIGHT ? 22 : 19), Math.PI)
                        .afterDisp(0.5-.4, robot.outtake.raise())
                        .afterDisp(5-3, blue() ? robot.outtake.goToLeft() : robot.outtake.goToRight())
                        .build();
            }
            if(robot.drive.pose.position.x > 0)
                return robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToSplineHeading(backdropPose, Math.toRadians(180))
                        .afterDisp(1, new SequentialAction(robot.outtake.raise(), new SleepAction(.5+.3), armMovement))
                        .build();
            else if(pickFromStack)
                return robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToSplineHeading(newVector(robot.drive.pose.position.x, trussBack.y-4), Math.PI)
                        .strafeTo(trussBack)
                        .strafeToSplineHeading(backdropPose, Math.PI)
                        .afterDisp(1, new SequentialAction(robot.outtake.raise(), armMovement))
                        .build();
            else
                return robot.drive.actionBuilder(robot.drive.pose)
                        .strafeTo(trussFront)
                        .turnTo(Math.PI)
                        .strafeTo(trussBack)
                        .afterDisp(50, robot.outtake::raise)
                        .strafeToSplineHeading(backdropPose, Math.PI)
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
                        .strafeToSplineHeading(trussBack.plus(new Vector2d(0, blue() ? 0 : 6+4)), Math.PI)
                        .strafeToSplineHeading(near() ? stackPose : stackPose.plus(newVector(-2,4)), Math.PI)
                        .afterDisp(36, () -> robot.intake.in())
                        .build();
            else return robot.drive.actionBuilder(robot.drive.pose)
//                    .turnTo(Math.PI)
                    .strafeToSplineHeading(stackPose, Math.PI)
                    .afterDisp(.8 * Math.abs(robot.drive.pose.position.minus(stackPose).norm()), () -> robot.intake.in())
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
                    .afterTime(.1, robot.outtake.lower())
                    .strafeToSplineHeading(new Vector2d(71+1, parkPose.y), Math.PI)
                    .build();
        }
        public Action generateResetTraj() {
            return robot.drive.actionBuilder(robot.drive.pose)
                    .strafeToSplineHeading(startPose.position, startPose.heading.toDouble())
                    .build();
        }
    }

    private boolean left() {
        return propLocation == Location.LEFT;
    }

    private boolean center() {
        return propLocation == Location.CENTER;
    }

    private boolean near() {
        return side == Side.NEAR;
    }

    private boolean blue() {
        return alliance == Alliance.BLUE;
    }

    private Trajectories trajectories;

    @Override
    public void runOpMode() {
        initialize();
        initLoop();
        if(!opModeIsActive()) return;
        double initTime = getRuntime();
        resetRuntime();
        runTimer.start();
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
                moveToBackdrop(true);
                placeOnBackdrop();
            } else {
                if(useAprilTags) {
                    Actions.runBlocking(new ParallelAction(
                            trajectories.generateToAprilTagDetection(),
                            robot.outtake.raise()
                    ));
                    updatePoseFromAprilTag();
                }
                moveToBackdrop(true);
                placeOnBackdrop();
                if(debugMode) {
                    pause();
                    if(!opModeIsActive()) return;
                }
                placePurplePixel();
                Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)
                    .lineToX(robot.drive.pose.position.x + 5)
                    .build()
                );
            }
        } else if(side == Side.FAR) {
            placePurplePixel();
//            if(debugMode) pause();
            if(pickFromStack) {
                pickFromStack();
//                if(debugMode) pause();
                if(!opModeIsActive()) return;
                Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToSplineHeading(newVector(robot.drive.pose.position.x+5, trajectories.trussBack.y), Math.PI)
                        .strafeTo(trajectories.trussBack)
                        .build());
                if(debugMode) {
                    pause();
                    if(!opModeIsActive()) return;
                }
            } else {
                Actions.runBlocking(new SequentialAction(
                        robot.drive.actionBuilder(robot.drive.pose)
                            .strafeTo(trajectories.trussFront)
                            .turnTo(Math.PI)
                            .waitSeconds(wait ? getRuntime() - WAIT_TIME : .01)
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
//        if(debugMode) pause();

        if(pickFromStack) {
            pickFromStack();
//            if(debugMode) pause();
            if(!opModeIsActive()) return;
            if(placeOnBackdrop) {
                moveToBackdrop(false);
                placeOnBackdrop();
            } else {
                placeInBackstage();
            }
        }

        status.setValue("parking at %.1fs", getRuntime());
        telemetry.update();
        Actions.runBlocking(trajectories.generateParkTraj());

        status.setValue("parked in %d s", runTimer.elapsedTime());
        telemetry.update();
//        TeleOp.setStartPose(robot.drive.pose);

        Actions.runBlocking(new SleepAction(3));
        if(debugMode) {
            telemetry.log().add("press to reset");
            pause();
            if(!opModeIsActive()) return;
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
        telemetry.addData("runtime", "%d", runTimer::elapsedTime);
    }

    private void placePurplePixel() {
        Actions.runBlocking(new ParallelAction(
                trajectories.generateSpikeMarkTraj(),
                robot.outtake.lower()
        ));
        robot.autoClaw.up();
        Actions.runBlocking(new SleepAction(0.8));
        if((alliance == Alliance.BLUE && side == Side.NEAR && propLocation == Location.LEFT) || (alliance == Alliance.RED && side == Side.NEAR && propLocation == Location.RIGHT)) {
            Pose2d start = robot.drive.pose;
            Actions.runBlocking(robot.drive.actionBuilder(start)
                .strafeTo(new Vector2d(start.position.x, start.position.y + 7 * (alliance == Alliance.BLUE ? 1 : -1)))
                .turnTo(Math.PI)
                .build());
        } else if(alliance == Alliance.RED && side == Side.FAR && propLocation != Location.LEFT)
            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)
                    .lineToX(robot.drive.pose.position.x - 5).build());
//        if(debugMode) pause();
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
                new SleepAction(0.5),
                new InstantAction(() -> robot.outtake.releaser.open()),
                new SleepAction(.3),
                new InstantAction(() -> robot.outtake.extender.goToMaxPos())
//                robot.outtake.motor.goToPosition(300, .4)
//                new SleepAction(0.3)
        ));
    }
    private Action getArmMovementAction() {
        switch (propLocation) {
            case LEFT:
                return robot.outtake.goToLeft();
            case RIGHT:
                return robot.outtake.goToRight();
//            case CENTER:
//                if(side == Side.FAR)
//                    return robot.outtake.extender.goToMaxPosWithActions();
            default:
                return new NullAction();
        }
    }
    private void pickFromStack() {
        status.setValue("moving to stack");
        telemetry.update();
        Actions.runBlocking(new ParallelAction(
                trajectories.generateToStack(),
                new SequentialAction(
                        new SleepAction(0.4),
                        robot.prepareForIntake()
                )
        ));
        if(debugMode) {
            robot.intake.stop();
            pause();
            if(!opModeIsActive()) return;
        }
        robot.intake.in();
        Pose2d start = robot.drive.pose;
        Actions.runBlocking(robot.drive.actionBuilder(start)
//                .strafeTo(start.position.plus(newVector(-10+1, 3)))
                .strafeTo(start.position.plus(newVector(-6, 22)))
                .strafeTo(start.position.plus(newVector(-3, 8)))
//                .afterDisp(5, () -> robot.intake.out())
                .build()
        );
        if(debugMode) {
            robot.intake.stop();
            pause();
            if(!opModeIsActive()) return;
        }
        robot.intake.out();
        executorService.schedule(() -> robot.intake.in(), 250, TimeUnit.MILLISECONDS);
        executorService.schedule(() -> robot.intake.out(), 1500, TimeUnit.MILLISECONDS);
        executorService.schedule(() -> robot.intake.in(), 1800, TimeUnit.MILLISECONDS);
        executorService.schedule(() -> robot.intake.out(), 2500, TimeUnit.MILLISECONDS);
        executorService.schedule(robot.intake::stop, 2500+1000, TimeUnit.MILLISECONDS);
    }
    private void placeInBackstage() {
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.pose)
                .strafeToSplineHeading(newVector(robot.drive.pose.position.x+5, trajectories.trussFront.y), Math.PI)
                .strafeTo(new Vector2d(5, trajectories.trussBack.y))
                .strafeToSplineHeading(newVector( trajectories.backdropPose.x-1, propLocation == Location.RIGHT ? 22 : 18), Math.PI)
                .afterDisp(5, robot.outtake.raise())
                .build()
        );
        robot.outtake.releaser.open();
        Actions.runBlocking(new SleepAction(.5));
    }

    private void pause() {
        runTimer.pause();
        status.setValue("paused (press a)");
        telemetry.update();
        while(!gamepad1.a && !gamepad2.a && opModeIsActive());
        if(!opModeIsActive()) return;
        status.setValue("resuming");
        telemetry.update();
        runTimer.resume();
    }

    private void initialize() {
        status = telemetry.addData("Status", "initializing...").setRetained(true);
        telemetry.update();

        robot = new Robot(hardwareMap);
        robot.outtake.releaser.open();
        try {
            propDetector = new TensorFlowObjectDetector(hardwareMap);
        } catch(Exception e) {
            telemetry.log().add("tensorflow init failed \n" + e);
        }
        propLocation = Location.LEFT;
        runTimer = new Timing.Timer(60);
        timer = new ElapsedTime();
        executorService = Executors.newSingleThreadScheduledExecutor();

        driver = new GamepadEx(gamepad1);
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

        if((gamepad1.start && gamepad1.back) || (driver.isDown(Button.START) && driver.isDown(Button.BACK)))
            requestOpModeStop();

        // ALLIANCE
        if(driver.isDown(Button.X) && !driver.isDown(Button.START) && !driver.isDown(Button.BACK)) alliance = Alliance.BLUE;
        else if(driver.isDown(Button.B) && !driver.isDown(Button.START) && !driver.isDown(Button.BACK)) alliance = Alliance.RED;

        // SIDE
        if(driver.isDown(Button.A) && !driver.isDown(Button.START)) {
            side = Side.NEAR;
            wait = false;
            parkPosition = ParkPosition.CORNER;
        } else if(driver.isDown(Button.Y) && !driver.isDown(Button.START) && !driver.isDown(Button.BACK)) {
            side = Side.FAR;
            wait = true;
        }

        // WAIT
        if(driver.wasJustReleased(Button.RIGHT_BUMPER) && !driver.isDown(Button.BACK)) wait = !wait;

        // GO THROUGH STAGE DOOR
        if(!driver.isDown(Button.BACK)) {
            double lsy = driver.getLeftY();
            if (lsy > .5) goThroughStageDoor = true;
            else if (lsy < -.5) goThroughStageDoor = false;
        }

        // PLACE ON BACKDROP
        if (lt.wasJustReleased() && !driver.isDown(Button.BACK)) placeOnBackdrop = !placeOnBackdrop;

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
        if(!driver.isDown(Button.BACK)) {
            if (driver.getRightY() < -.1)
                parkPosition = ParkPosition.CENTER;
            else if (driver.getRightY() > .1)
                parkPosition = ParkPosition.CORNER;
        }

        // DEBUG MODE
        if(driver.isDown(Button.START) && driver.isDown(Button.Y)) debugMode = true;
        else if(driver.isDown(Button.START) && driver.isDown(Button.X)) debugMode = false;

        // PROP LOCATION OVERRIDE
        if (driver.isDown(Button.DPAD_LEFT) || driver.isDown(Button.DPAD_UP) || driver.isDown(Button.DPAD_RIGHT)) {
            propLocationOverride = true;
            if (driver.isDown(Button.DPAD_LEFT))
                propLocation = Location.LEFT;
            else if (driver.isDown(Button.DPAD_UP))
                propLocation = Location.CENTER;
            else
                propLocation = Location.RIGHT;
            propLocationTelemetry.setValue(propLocation + " (override)");
        } else if (driver.wasJustReleased(Button.BACK) && back.getTimeDown() < .3)
            propLocationOverride = false;

        // PRELOAD
        if(driver.isDown(Button.BACK)) {
            if (driver.wasJustPressed(Button.A) && !driver.isDown(Button.START)) robot.outtake.releaser.open();
            else if(driver.wasJustPressed(Button.B) && !driver.isDown(Button.START)) robot.outtake.releaser.close();
            if(driver.isDown(Button.Y)) robot.autoClaw.up(.01);
            else if(driver.isDown(Button.X)) robot.autoClaw.down(.01);
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
        Actions.runBlocking(trajectories.generateToAprilTagDetection());

        final double MAX_DETECTION_TIME = 1.5, MAX_CHANGE = 15+7;
        status.setValue("looking for april tags...");
        telemetry.update();
        timer.reset();
        AprilTagDetection detectedTag = null;
        while(detectedTag == null && timer.seconds() < MAX_DETECTION_TIME) {
            detectedTag = aprilTagDetector.detect();
        }
        telemetry.log().add("");
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
        Vector2d fieldTagPose = newVector(aprilTagDetector.aprilTagIDs.getPose(detectedTag.id));
        telemetry.log().add("april tag pose (field): %s", vectorToString(fieldTagPose));
        Pose2d newRobotPose = new Pose2d(fieldTagPose.x - cameraTagPose.x, fieldTagPose.y + cameraTagPose.y, robot.drive.pose.heading.toDouble());
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
