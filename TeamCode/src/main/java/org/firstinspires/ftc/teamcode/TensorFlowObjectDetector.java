package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.Location;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class TensorFlowObjectDetector {
    private static final String MODEL_ASSET = "blue_red_prop_model_3.tflite";
    private static final String[] LABELS = { "blueProp", "redProp" };
    private static final float MIN_CONFIDENCE = .5f;
    private static final double CENTER_DIVISION = 350;
    private static final double MAX_SIZE = 250;
    private final TfodProcessor tfod; // stores instance of TFOD processor
    public VisionPortal visionPortal; // stores instance of vision portal
    private List<Recognition> recognitions;
    private Recognition mostConfidentRecognition, previousRecognition;
    ElapsedTime previousRecognitionTimer;
    private Alliance alliance;
    public TensorFlowObjectDetector(HardwareMap hardwareMap) {
        tfod = new TfodProcessor.Builder() // create the TF processor using a builder
                .setModelAssetName(MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder(); // create vision portal using a builder
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")); // set the camera
        builder.addProcessor(tfod); // set and enable processor
        visionPortal = builder.build(); // build the vision portal using the above settings
        tfod.setMinResultConfidence(MIN_CONFIDENCE); // set confidence threshold for TFOD recognitions
        previousRecognitionTimer = new ElapsedTime();
    }
    public void update() {
        recognitions = tfod.getRecognitions();

        if(recognitions.size() != 0) {
            mostConfidentRecognition = recognitions.get(0);
            if(!isValid(mostConfidentRecognition)) { // if invalid
                if(recognitions.size() > 1) { // set to next one if possible
                    mostConfidentRecognition = recognitions.get(1);
                } else // if no others set to null
                    mostConfidentRecognition = null;
            }
            if(mostConfidentRecognition != null) { // if not null must be valid
                float maxConfidence = mostConfidentRecognition.getConfidence();
                for (Recognition recognition : recognitions) {
                    if (recognition.getConfidence() > maxConfidence && isValid(recognition))
                        mostConfidentRecognition = recognition;
                }
                previousRecognition = mostConfidentRecognition;
                previousRecognitionTimer.reset();
            }
        } else {
            mostConfidentRecognition = null;
        }
    }

    public boolean isValid(Recognition recognition) {
        return isRightColor(recognition) && !isTooBig(recognition);
    }
    private boolean isRightColor(Recognition recognition) {
        if(alliance == Alliance.BLUE)
            return recognition.getLabel().equals("blueProp");
        else
            return recognition.getLabel().equals("redProp");
    }
    private boolean isTooBig(Recognition recognition) {
        return recognition.getWidth() > MAX_SIZE || recognition.getHeight() > MAX_SIZE;
    }

    public Location getLocation(Recognition recognition)
    {
        // if it doesn't see anything it must be left
        if(recognition == null)
            return Location.LEFT;

        double centerX = (recognition.getLeft() + recognition.getRight()) / 2;
        if(centerX > CENTER_DIVISION)
            return Location.RIGHT;
        else
            return Location.CENTER;
    }
    public Location getLocation() {
        return getLocation(mostConfidentRecognition);
    }

    // sends info to telemetry about all found objects
    public void telemetryAll(Telemetry telemetry) {
        if(recognitions.size() == 0)
            telemetry.addLine("No objects detected");
        else
            telemetry.addData("# Objects Detected", recognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : recognitions) {
            telemetry.addLine();
            telemetrySingle(telemetry, recognition);
        }
        if(recognitions.size() == 0 && previousRecognition != null) {
            telemetry.addData("\nPreviously seen", "%.1f seconds ago", previousRecognitionTimer.seconds());
            telemetrySingle(telemetry, previousRecognition);
        }
    }
    // sends info to telemetry about the single best object
    public void telemetryBest(Telemetry telemetry) {
        if(mostConfidentRecognition != null) {
            telemetrySingle(telemetry, mostConfidentRecognition);
        } else {
            telemetry.addLine("no objects found");
        }
    }
    public void telemetrySingle(Telemetry telemetry, Recognition recognition) {
        double x = (recognition.getLeft() + recognition.getRight()) / 2;
        double y = (recognition.getTop() + recognition.getBottom()) / 2;

        telemetry.addData("Found","%s (%.0f %% conf.) %s", recognition.getLabel(), recognition.getConfidence() * 100, isRightColor(recognition) ? "" : "(wrong color)");
        telemetry.addData("- Position", "%s (%.0f, %.0f)", getLocation(recognition), x, y);
        telemetry.addData("- Size", "%.0f x %.0f %s", recognition.getWidth(), recognition.getHeight(), isTooBig(recognition) ? "(too big)" : "");
    }
    public void stopDetecting() { // save CPU resources when camera is no longer needed
        visionPortal.close();
        tfod.shutdown();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }
}
