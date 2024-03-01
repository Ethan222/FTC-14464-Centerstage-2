package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CustomButton;

public class CustomServo {
    public final String name;
    private final Servo servo;
    private final double minPosition, maxPosition;
    private CustomButton button1, button2;
    public static final double downTime = .4, doubleTapTime = .15;
    public static final double singleRotateAmount = .02, continuousRotateAmount = .004;
    private static final double INCREMENT = .01;

    public CustomServo(String name, HardwareMap hardwareMap, String id, double minPos, double maxPos) {
        this.name = name;
        servo = hardwareMap.get(Servo.class, id);
        minPosition = minPos;
        maxPosition = maxPos;
    }
    public void setPosition(double pos) {
        servo.setPosition(Range.clip(pos, minPosition, maxPosition));
    }
    public void rotateBy(double change) {
        setPosition(getPosition() + change);
    }
    public void setButtons(CustomButton b1, CustomButton b2) {
        button1 = b1;
        button2 = b2;
    }
    public void update() {
        button1.update();
        button2.update();
        if (button1.getState() == CustomButton.State.JUST_DOWN) {
            if(button1.getTimeUp() < doubleTapTime)
                setPosition(maxPosition);
            else
                rotateBy(singleRotateAmount);
        } else if (button1.getState() == CustomButton.State.DOWN && button1.getTimeDown() > downTime)
            rotateBy(continuousRotateAmount);
        else if (button2.getState() == CustomButton.State.JUST_DOWN) {
            if(button2.getTimeUp() < doubleTapTime)
                setPosition(minPosition);
            else rotateBy(-singleRotateAmount);
        } else if(button2.getState() == CustomButton.State.DOWN && button2.getTimeDown() > downTime)
            rotateBy(-continuousRotateAmount);
    }

    public double getPosition() {
        return servo.getPosition();
    }
    public String getTelemetry() {
        return String.format("%.3f", getPosition());
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

    public double getIncrement() {
        return INCREMENT;
    }
    public void rotateIncrementally() {
        rotateBy(getIncrement());
    }
    public void unrotateIncrementally() {
        rotateBy(-getIncrement());
    }
}
