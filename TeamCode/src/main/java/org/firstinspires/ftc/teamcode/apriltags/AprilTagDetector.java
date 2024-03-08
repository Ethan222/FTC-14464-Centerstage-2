package org.firstinspires.ftc.teamcode.apriltags;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.Location;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

public class AprilTagDetector {
    public static final double INCHES_PER_METER = 39.3701;
    private final static double tagsize = .048; // meters
    // lens intrinsics (units are pixels)
    private static final double fx = 890.109, fy = 890.109, cx = 486.52, cy = 235.57; // c270 webcam at 800x448

    private final OpenCvCamera camera;
    private final AprilTagDetectionPipeline pipeline;
    private List<AprilTagDetection> currentDetections;
    public AprilTagIDs aprilTagIDs;
    private Backdrop backdrop;

    public AprilTagDetector(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) { }
        });
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        aprilTagIDs = new AprilTagIDs();
    }
    public static Vector2d convertPose(AprilTagPose pose) {
        return new Vector2d(pose.z * INCHES_PER_METER, pose.x * INCHES_PER_METER);
    }
    public static String tagPoseToString(AprilTagPose pose) {
        return String.format("(%.2f, %.2f, %.2f) in", pose.x, pose.y, pose.z);
    }

    public void setAlliance(Alliance alliance) {
        backdrop = AprilTagIDs.getBackdrop(alliance);
    }
    public void update() {
        currentDetections = pipeline.getLatestDetections();
    }
    public List<AprilTagDetection> getCurrentDetections() {
        update();
        return currentDetections;
    }
    public AprilTagDetection detect() {
        update();
        if(currentDetections.size() == 0) return null;
        return currentDetections.get(0);
    }
    public List<Location> getDetectedLocations() {
        List<Location> ret = new ArrayList<>();
        update();
        for(AprilTagDetection tag : currentDetections) {
            ret.add(backdrop.getLocation(tag.id));
        }
        return ret;
    }
}