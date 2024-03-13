package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class Outtake {
    public final Motor motor;
    public final Flipper flipper;
    public final CustomServo extender;
    public final Rotator armRotator, pixelRotator;
    public final Releaser releaser;
    private final ScheduledExecutorService executorService;
    private Action lowerAction;
    public Outtake(HardwareMap hardwareMap, String motorName, String flipperName, String extenderName, String armRotatorName, String pixelRotatorName, String releaserName) {
        motor = new Motor(hardwareMap, motorName);
        motor.brakeOnZeroPower();
        flipper = new Flipper(hardwareMap, flipperName, 0.5, 1);
        extender = new CustomServo(hardwareMap, extenderName, .3, .88);
        armRotator = new Rotator(hardwareMap, armRotatorName, 0, 1, .5, .007);
        pixelRotator = new Rotator(hardwareMap, pixelRotatorName, 0, 1, .72, .004);
        releaser = new Releaser(hardwareMap, releaserName, .6, 1);
        executorService = Executors.newSingleThreadScheduledExecutor();
        extender.goToMinPos();
        center();
    }
    public Action raise() {
        armRotator.setCenterPos();
        return new SequentialAction(
                extender.goToMinPosWithActions(),
                flipper.flip(),
                new InstantAction(this::center)
        );
    }
    public Action raiseAndExtendSlightly() {
        return new SequentialAction(
                flipper.flip(),
                new InstantAction(this::center),
                extender.goToPos(.87)
        );
    }
    public Action lower() {
        center();
        lowerAction = new ParallelAction(
//                motor.goToPreset(0, .4),
                extender.goToMinPosWithActions(),
                flipper.unflip(.2)
        );
        return lowerAction;
    }
    public Class getLowerActionClass() {
        return lowerAction == null ? null : lowerAction.getClass();
    }
    public void center() {
        armRotator.center();
        pixelRotator.center();
    }
    public void release() {
        releaser.open();
        executorService.schedule(releaser::close, 500, TimeUnit.MILLISECONDS);
    }
    public Action goToLeft() {
        return new SequentialAction(
                extender.goToPos(.7),
                new ParallelAction(
                        armRotator.goToPos(0.2),
                        pixelRotator.goToPos(0.45+.1)
                )
        );
    }
    public Action goToRight() {
        return new ParallelAction(
            extender.goToPos(0.6-.2),
            armRotator.goToPos(.7),
            pixelRotator.goToPos(0.85)
        );
    }
    public void moveRight() {
        armRotator.rotateIncrementally();
        pixelRotator.rotateIncrementally();
    }
    public void moveLeft() {
        armRotator.unrotateIncrementally();
        pixelRotator.unrotateIncrementally();
    }

    public static class Rotator extends CustomServo {
        public static final String LEFT = "LEFT", CENTER = "CENTER", RIGHT = "RIGHT";
        private double CENTER_POS;
        private final double INCREMENT;
        public Rotator(HardwareMap hardwareMap, String id, double minPos, double maxPos, double centerPos, double increment) {
          super(hardwareMap, id, minPos, maxPos);
          CENTER_POS = centerPos;
          INCREMENT = increment;
        }
        public void center() {
            rotateIncrementally();
            setPosition(CENTER_POS);
        }
        public void setCenterPos() {
          CENTER_POS = getPosition();
        }
        @Override public double getIncrement() { return INCREMENT; }
        @Override public String getState() {
            String state = super.getState();
            if(Objects.equals(state, CustomServo.MIN))
                return LEFT;
            else if(Objects.equals(state, CustomServo.MAX))
                return RIGHT;
            else if(Math.abs(getPosition() - CENTER_POS) < .01)
                return CENTER;
            else return "";
        }
    }
    public static class Releaser extends CustomServo {
        public static final String OPEN = "OPEN", CLOSED = "CLOSED";
        public Releaser(HardwareMap hardwareMap, String id, double minPos, double maxPos) {
            super(hardwareMap, id, minPos, maxPos);
        }
        public void open() {
            goToMinPos();
        }
        public void close() {
            goToMaxPos();
        }

        @Override
        public String getState() {
            double pos = getPosition();
            final double ERROR = .01;
            if(Math.abs(pos - getMinPos()) < ERROR)
                return OPEN;
            else if(Math.abs(pos - getMaxPos()) < ERROR)
                return CLOSED;
            else return "";
        }
    }
}
