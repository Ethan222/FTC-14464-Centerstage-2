package org.firstinspires.ftc.teamcode.apriltags;

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

@Config
public class AprilTagDetector {
    public static double INCHES_PER_METER = 3.28084*4;
    public static double tagsize = .166; // meters TODO: check this
    // lens intrinsics (units are pixels)
    // this calibration is for C920 webcame at 800x448
    public static double fx = 578.272, fy = 578.272, cx = 402.145, cy = 221.506;

    private OpenCvCamera camera;
    private AprilTagDetectionPipeline pipeline;
    private List<AprilTagDetection> currentDetections;
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
    public AprilTagDetection detect(Backdrop backdrop) {
        update();
        if(currentDetections.size() == 0) return null;

        for(AprilTagDetection tag : currentDetections) {
            if(backdrop.contains(tag.id))
                return tag;
        }
    }
    public AprilTagDetection detect() {
        return detect(backdrop);
    }
    public List<Location> getDetectedLocations() {
        List<Location> ret = new ArrayList<>();
        update();
        for(AprilTagDetection tag : currentDetections) {
            ret.add(backdrop.getLocation(tag));
        }
        return ret;
    }
}