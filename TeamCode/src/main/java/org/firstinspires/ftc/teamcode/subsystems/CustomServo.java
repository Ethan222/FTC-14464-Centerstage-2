package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

// class to control a servo
public class CustomServo {
    // static fields (the same for all CustomServo objects)
    public static final String MIN = "MIN", MAX = "MAX";    // constant strings to represent minimum and maximum states
    private static final double DEFAULT_SPEED = .01;        // constant value for default speed of a servo

    // instance variables (unique for each servo)
    private final Servo servo;      // the actual servo
    private final double minPosition, maxPosition;      // minimum and maximum positions of the servo

    // constructors
    public CustomServo(HardwareMap hardwareMap, String id, double minPos, double maxPos) {
        servo = hardwareMap.get(Servo.class, id);   // initialize servo from the hardware map
        minPosition = minPos;   // set the min and max positions
        maxPosition = maxPos;
    }
    public CustomServo(HardwareMap hardwareMap, String id) {
        this(hardwareMap, id, 0, 1);    // set default min and max positions to 0 and 1, respectively
    }

    // method to set the servo's position
    public void setPosition(double pos) {
        servo.setPosition(Range.clip(pos, minPosition, maxPosition));   // makes sure it's a valid position within the range
    }
    public void rotateBy(double change) {
        setPosition(getPosition() + change);
    }

    public double getPosition() {
        return servo.getPosition();
    }
    public String getTelemetry() {
        return String.format("%.2f (%s)", getPosition(), getState());
    }

    public String getState() {
        double psn = getPosition();
        final double tolerance = .01;
        if(Math.abs(psn - minPosition) < tolerance)
            return MIN;
        else if(Math.abs(psn - maxPosition) < tolerance)
            return MAX;
        else return "";
    }

    public double getMinPos() {
        return minPosition;
    }
    public double getMaxPos() {
        return maxPosition;
    }

    public void goToMaxPos() {
        setPosition(maxPosition);
    }

    public void goToMinPos() {
        setPosition(minPosition);
    }
    public Action goToMaxPosWithActions() {
        return goToPos(maxPosition);
    }
    public Action goToMinPosWithActions() {
        return goToPos(minPosition);
    }

    public double getIncrement() {
        return DEFAULT_SPEED;
    }
    public void rotateIncrementally() {
        rotateBy(getIncrement());
    }
    public void unrotateIncrementally() {
        rotateBy(-getIncrement());
    }

    public Action goToPos(double pos, double speed) {
        return telemetryPacket -> {
            if(Math.abs(pos - getPosition()) < .01)
                return false;
            rotateBy(pos > getPosition() ? speed : -speed);
            return true;
        };
    }
    public Action goToPos(double pos) {
        return goToPos(pos, DEFAULT_SPEED);
    }
}
