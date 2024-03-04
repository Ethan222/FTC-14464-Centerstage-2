package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class CustomServo {
    private static final double DEFAULT_SPEED = .01;
    private final Servo servo;
    private final double minPosition, maxPosition;
    private static final double INCREMENT = .01;

    public CustomServo(HardwareMap hardwareMap, String id, double minPos, double maxPos) {
        servo = hardwareMap.get(Servo.class, id);
        minPosition = minPos;
        maxPosition = maxPos;
    }

    public CustomServo(HardwareMap hardwareMap, String id) {
        this(hardwareMap, id, 0, 1);
    }

    public void setPosition(double pos) {
        servo.setPosition(Range.clip(pos, minPosition, maxPosition));
    }
    public void rotateBy(double change) {
        setPosition(getPosition() + change);
    }

    public double getPosition() {
        return servo.getPosition();
    }
    public String getTelemetry() {
        return String.format("%.2f", getPosition());
    }

    public double getMinPos() {
        return minPosition;
    }

    public void goToMaxPos() {
        setPosition(maxPosition);
    }

    public void goToMinPos() {
        setPosition(minPosition);
    }
    public Action goToMinPosWithActions() {
        return goToPos(minPosition);
    }

    public double getIncrement() {
        return INCREMENT;
    }
    public void rotateIncrementally() {
        rotateBy(getIncrement());
    }
    public void unrotateIncrementally() {
        rotateBy(-getIncrement());
    }

    public Action goToPos(double pos, double speed) {
        return telemetryPacket -> {
            if(Math.abs(pos - getPosition()) < .001)
                return false;
            rotateBy(pos > getPosition() ? speed : -speed);
            return true;
        };
    }
    public Action goToPos(double pos) {
        return goToPos(pos, DEFAULT_SPEED);
    }
}
