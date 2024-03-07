/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auto.AprilTags;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.enums.Alliance;
import org.firstinspires.ftc.teamcode.auto.enums.Location;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@TeleOp(name = "Test April Tag Detection", group = "test")
public class TestAprilTagDetection extends LinearOpMode
{
    Robot robot;
    AprilTagDetector aprilTagDetector;

    final static double WHEEL_SPEED = .3;

    enum State {
        DETECTING, AT_BACKDROP
    }
    State state = State.DETECTING;

    static Backdrop backdrop = AprilTagIDs.getBackdrop(Alliance.BLUE);
    static Location location = Location.LEFT;
    int tagIdToDetect;
    AprilTagDetection detectedTag = null;
    boolean currentlyDetecting = false;
    double timeTo1stDetection = 0;
    double x = 0, y = 0;
    Pose2d startPose;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap);
        robot.outtake.releaser.close();

        aprilTagDetector = new AprilTagDetector(hardwareMap);

        telemetry.setMsTransmissionInterval(50);

        while(!isStopRequested() && !(gamepad1.start && gamepad1.back)) {
            if(state == State.DETECTING) {
                telemetry.addData("Claw (RB/LB)", robot.claw1.getState());
                telemetry.addLine();
                updateDrive();
                updateAlliance();
                detectAprilTags();
                if (gamepad1.right_bumper)
                    robot.claw1.down();
                else if (gamepad1.left_bumper)
                    robot.claw1.up();
                if(gamepad1.a && !gamepad1.start)
                    goToBackdrop();
            } else if(state == State.AT_BACKDROP) {
                telemetry.addData("Placed on", location);
                telemetry.addLine("Press b to reset");
                if(gamepad1.b && !gamepad1.start)
                    reset();
            }
            if(!gamepad2.atRest())
                telemetry.addLine("\nSwitch to gamepad 1");
            telemetry.update();
        }
    }
    void updateDrive() {
        robot.drive.setWeightedDrivePower(
                new Pose2d(
                        WHEEL_SPEED * -gamepad1.left_stick_y,
                        WHEEL_SPEED * -gamepad1.left_stick_x,
                        WHEEL_SPEED * -gamepad1.right_stick_x
                )
        );
        robot.drive.update();
    }
    void updateAlliance() {
        if(gamepad1.x)
            aprilTagDetector.setAlliance(Alliance.BLUE);
        else if(gamepad1.b && !gamepad1.start)
            aprilTagDetector.setAlliance(Alliance.RED);
    }
    void detectAprilTags() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetector.getCurrentDetections();
        currentlyDetecting = false;
        if(currentDetections.size() != 0)
        {
            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == tagIdToDetect) {
                    detectedTag = tag;
                    currentlyDetecting = true;
                    if(timeTo1stDetection == 0)
                        timeTo1stDetection = getRuntime();
                }
            }
        }

        telemetry.addData("Looking for", "%s %s (id %d)", backdrop.alliance, location, tagIdToDetect);
        if(detectedTag == null) {
            if (currentDetections.size() != 0)
                telemetry.addLine("not detecting it");
            else
                telemetry.addLine("not detecting anything");
        } else {
            tagToTelemetry(detectedTag);
            telemetry.addData("currently detecting", currentlyDetecting);
        }
        if(timeTo1stDetection != 0)
            telemetry.addData("Time to 1st detection", "%.1f s", timeTo1stDetection);

        if(detectedTag != null) {
            x = detectedTag.pose.z*INCHES_PER_METER - 1.7;
            y = -detectedTag.pose.x*INCHES_PER_METER - 6;
        }
        telemetry.addData("\nPress a to move", "\tx = %.1f in. forward,\n\t\t\t\t\t\t\ty = %.1f in. %s", x, Math.abs(y), y > 0 ? "left" : "right");
    }
    void goToBackdrop() {
        startPose = robot.drive.getPoseEstimate();
        robot.claw1.down();
        robot.outtake.goToUpPosition();
        robot.outtake.rotator.rotateFully();
        robot.drive.followTrajectorySequence(robot.drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(x, y), .3)
                .forward(2, MecanumDrive.getVelocityConstraint(2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), MecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build()
        );
        robot.claw1.up();
        robot.claw2.up();
        robot.outtake.goToPosition(robot.outtake.getPosition() + 350, .1);
        state = State.AT_BACKDROP;
    }
    void reset() {
        robot.drive.followTrajectorySequence(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .addTemporalMarker(.2, () -> {
                    robot.outtake.rotator.retractFully();
                })
                .addTemporalMarker(.7, () -> {
                    robot.outtake.goToDownPosition();
                })
                .back(1)
                .splineToConstantHeading(new Vector2d(startPose.getX(), startPose.getY()), Math.PI)
                .build()
        );
        state = State.DETECTING;
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        double x = detection.pose.x * INCHES_PER_METER;
        double y = detection.pose.y * INCHES_PER_METER;
        double z = detection.pose.z * INCHES_PER_METER;
        telemetry.addData("Position", "(%.2f, %.2f, %.2f) in.", x, y, z);
    }
}