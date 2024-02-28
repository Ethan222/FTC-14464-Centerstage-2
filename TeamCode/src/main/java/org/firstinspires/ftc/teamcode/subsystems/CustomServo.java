package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CustomButton;

public class CustomServo {
    public final ServoEx servo;
    private CustomButton button1, button2;
    private double minPosition, maxPosition;
    public static final double downTime = .4, doubleTapTime = .1;
    public static final double singleRotateAmount = .02, continousRotateAmount = .001;
    public CustomServo(HardwareMap hardwareMap, String name, double minPos, double maxPos) {
        servo = new SimpleServo(hardwareMap, name, 0, 360);
        minPosition = minPos;
        maxPosition = maxPos;
    }
    public void setPosition(double pos) {
        servo.setPosition(Range.clip(pos, minPosition, maxPosition));
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
                servo.rotateBy(singleRotateAmount);
        } else if (button1.getState() == CustomButton.State.DOWN && button1.getTimeDown() > downTime)
            servo.rotateBy(continousRotateAmount);
        else if (button2.getState() == CustomButton.State.JUST_DOWN) {
            if(button2.getTimeUp() < doubleTapTime)
                setPosition(minPosition);
            else servo.rotateBy(-singleRotateAmount);
        } else if(button2.getState() == CustomButton.State.DOWN && button2.getTimeDown() > downTime)
            servo.rotateBy(-continousRotateAmount);
    }

    public double getPosition() {
        return servo.getPosition();
    }
}
