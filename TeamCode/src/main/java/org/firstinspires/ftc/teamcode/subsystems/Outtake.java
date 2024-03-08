package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
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
    public Outtake(HardwareMap hardwareMap, String motorName, String flipperName, String extenderName, String armRotatorName, String pixelRotatorName, String releaserName) {
        motor = new Motor(hardwareMap, motorName);
        flipper = new Flipper(hardwareMap, flipperName, .57, 1);
        extender = new CustomServo(hardwareMap, extenderName, .37-.2, .95);
        armRotator = new Rotator(hardwareMap, armRotatorName, 0, 1, .44, .007);
        pixelRotator = new Rotator(hardwareMap, pixelRotatorName, 0, 1,.6, .004);
        releaser = new Releaser(hardwareMap, releaserName, .6, 1);
        executorService = Executors.newSingleThreadScheduledExecutor();
        extender.goToMinPos();
        flipper.goToMinPos();
        center();
    }
    public Action raise() {
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
        return new ParallelAction(
                extender.goToMinPosWithActions(),
                flipper.unflip()
        );
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
        return new ParallelAction(
            extender.goToPos(.7),
            armRotator.goToPos(.2-.1),
            pixelRotator.goToPos(.4)
        );
    }
    public Action goToRight() {
        return new ParallelAction(
            extender.goToPos(.7),
            armRotator.goToPos(.7),
            pixelRotator.goToPos(.8)
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
