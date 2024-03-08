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

package org.firstinspires.ftc.teamcode.apriltags;

import static org.firstinspires.ftc.teamcode.apriltags.AprilTagDetector.INCHES_PER_METER;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.enums.Location;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Test April Tag Detection")
public class TestAprilTagDetection extends LinearOpMode
{
    Robot robot;
    AprilTagDetector aprilTagDetector;

    final static double WHEEL_SPEED = .3;

    enum State {
        DETECTING, AT_BACKDROP
    }
    State state = State.DETECTING;

    static Location location = Location.LEFT;
    double timeTo1stDetection = 0;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, new Pose2d(0, 0, 0), telemetry);
        robot.outtake.releaser.close();

        aprilTagDetector = new AprilTagDetector(hardwareMap);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();
        resetRuntime();
        while(!isStopRequested() && !(gamepad1.start && gamepad1.back)) {
            if(state == State.DETECTING) {
                updateDrive();
                detectAprilTags();
                if (gamepad1.right_bumper)
                    robot.outtake.releaser.close();
                else if (gamepad1.left_bumper)
                    robot.outtake.releaser.open();
//                if(gamepad1.a && !gamepad1.start)
//                    goToBackdrop();
            } else if(state == State.AT_BACKDROP) {
                telemetry.addData("Placed on", location);
                telemetry.addLine("Press b to reset");
//                if(gamepad1.b && !gamepad1.start)
//                    reset();
            }
            telemetry.update();
        }
    }
    void updateDrive() {
        robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));
        robot.drive.updatePoseEstimate();
    }
    void detectAprilTags() {
        List<AprilTagDetection> currentDetections = aprilTagDetector.getCurrentDetections();
        for(AprilTagDetection tag : currentDetections) {
            if(timeTo1stDetection == 0) {
                timeTo1stDetection = getRuntime();
                telemetry.addData("time to 1st detection", "%.2f s", getRuntime()).setRetained(true);
            }
            Object[] location = AprilTagIDs.getLocation(tag.id);
            if(location == null)
                telemetry.addData("detecting", "id %d (unknown location)", tag.id);
            else
                telemetry.addData("detecting", "%s %s (id %d)", location[0], location[1], tag.id);
            tagToTelemetry(tag);
            telemetry.addLine();
        }
    }
//    void goToBackdrop() {
//        startPose = robot.drive.getPoseEstimate();
//        robot.claw1.down();
//        robot.outtake.goToUpPosition();
//        robot.outtake.rotator.rotateFully();
//        robot.drive.followTrajectorySequence(robot.drive.trajectorySequenceBuilder(startPose)
//                .splineToConstantHeading(new Vector2d(x, y), .3)
//                .forward(2, MecanumDrive.getVelocityConstraint(2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), MecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build()
//        );
//        robot.claw1.up();
//        robot.claw2.up();
//        robot.outtake.goToPosition(robot.outtake.getPosition() + 350, .1);
//        state = State.AT_BACKDROP;
//    }
//    void reset() {
//        robot.drive.followTrajectorySequence(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
//                .addTemporalMarker(.2, () -> {
//                    robot.outtake.rotator.retractFully();
//                })
//                .addTemporalMarker(.7, () -> {
//                    robot.outtake.goToDownPosition();
//                })
//                .back(1)
//                .splineToConstantHeading(new Vector2d(startPose.getX(), startPose.getY()), Math.PI)
//                .build()
//        );
//        state = State.DETECTING;
//    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        double x = detection.pose.x * INCHES_PER_METER;
        double y = detection.pose.y * INCHES_PER_METER;
        double z = detection.pose.z * INCHES_PER_METER;
        telemetry.addData("Position", "(%.2f, %.2f, %.2f) in.", x, y, z);
    }
}