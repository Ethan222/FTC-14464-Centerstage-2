package org.firstinspires.ftc.teamcode;

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

@Autonomous(preselectTeleOp = "Tele")
public class Auto extends LinearOpMode {
    private enum ParkPosition {
        CORNER, CENTER
    }
    private static Alliance alliance = Alliance.BLUE;
    private static Side side = Side.NEAR;
    private static boolean wait = false, goThroughStageDoor = true, placeOnBackdrop = true, useAprilTags = false, pickFromStack = false;
    private static ParkPosition parkPosition = ParkPosition.CORNER;
    private static boolean debugMode = false;

    private Robot robot;
    private TensorFlowObjectDetector propDetector;
    private Location propLocation;
    private ElapsedTime timer;
    private ScheduledExecutorService executorService;
    private boolean initialized = false, propLocationOverride = false;
    private static final double MIN_INIT_TIME = 5.5, WAIT_TIME = 15;
    private Telemetry.Item status, propLocationTelemetry;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initLoop();
    }
    private void initialize() {
        status = telemetry.addData("Status", "initializing...").setRetained(true);
        telemetry.update();
        propDetector = new TensorFlowObjectDetector(hardwareMap);
        propLocation = Location.LEFT;
        timer = new ElapsedTime();
        executorService = Executors.newSingleThreadScheduledExecutor();
        telemetry.addLine().addData("\nalliance", () -> alliance).addData("side", () -> side);
        telemetry.addData("wait (RB/LB)", () -> wait);
        telemetry.addData("go through stage door (LS)", () -> (side == Side.FAR || pickFromStack) ? goThroughStageDoor : "n/a");
        telemetry.addData("place on backdrop (RT/LT)", () -> placeOnBackdrop);
        telemetry.addData("pick from stack (back + RT/LT)", () -> pickFromStack);
        telemetry.addData("use april tags (back + RB/LB)", () -> placeOnBackdrop ? useAprilTags : "n/a");
        telemetry.addData("park (RS)", ((placeOnBackdrop && (wait || pickFromStack)) ? "n/a" : parkPosition));
        propLocationTelemetry = telemetry.addData("\nprop location", null);
    }
    private void initLoop() {
        while(opModeInInit()) {
            getGamepadInput();

            if (!propLocationOverride) {
                detectTeamProp();
            }

            telemetry.update();
        }
    }

    private void getGamepadInput() {
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
        if(gamepad2.right_bumper && !gamepad2.back) wait = true;
        else if(gamepad2.left_bumper && !gamepad2.back) wait = false;

        // GO THROUGH STAGE DOOR
        if(gamepad2.left_stick_y < -.5) goThroughStageDoor = true;
        else if(gamepad2.left_stick_y > .5) goThroughStageDoor = false;

        // PLACE ON BACKDROP
        if (gamepad2.right_trigger > .1 && !gamepad2.back) placeOnBackdrop = true;
        else if(gamepad2.left_trigger > .1 && !gamepad2.back) placeOnBackdrop = false;

        // PICK FROM STACK
        if(gamepad2.right_trigger > .1 && gamepad2.back) {
            pickFromStack = true;
            wait = false;
            goThroughStageDoor = true;
        } else if(gamepad2.left_trigger > .1 && gamepad2.back) {
            pickFromStack = false;
            if(side == Side.FAR) wait = true;
        }

        // APRIL TAGS
        if(placeOnBackdrop) {
            if(gamepad2.right_bumper && gamepad2.back)
                useAprilTags = true;
            else if(gamepad2.left_bumper && gamepad2.back)
                useAprilTags = false;
        }

        // PARK
        if(gamepad2.right_stick_y < -.1)
            parkPosition = ParkPosition.CENTER;
        else if(gamepad2.right_stick_y > .1)
            parkPosition = ParkPosition.CORNER;

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
        } else if (gamepad2.back)
            propLocationOverride = false;
    }

    private void detectTeamProp() {
        propDetector.setAlliance(alliance);
        propDetector.update();
        propLocation = propDetector.getLocation();
        propLocationTelemetry.setValue(propLocation);
        telemetry.addLine();
        propDetector.telemetryAll(telemetry);
    }
}
