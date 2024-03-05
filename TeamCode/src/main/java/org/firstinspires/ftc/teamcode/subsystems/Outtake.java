package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
        extender = new CustomServo(hardwareMap, extenderName, .37, .95);
        armRotator = new Rotator(hardwareMap, armRotatorName, 0, 1, .44, .005);
        pixelRotator = new Rotator(hardwareMap, pixelRotatorName, 0, 1, .66, .008);
        releaser = new Releaser(hardwareMap, releaserName, .6, 1);
        executorService = Executors.newSingleThreadScheduledExecutor();
    }
    public Action raise() {
        return new SequentialAction(
                flipper.flip(),
                new InstantAction(this::center)
        );
    }
    public Action raiseAndExtendSlightly() {
        return new SequentialAction(
                flipper.flip(),
                new InstantAction(this::center),
                new InstantAction(() -> extender.goToPos(.6))
        );
    }
    public Action lower() {
        center();
        return new SequentialAction(
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
            armRotator.goToPos(.2),
            pixelRotator.goToPos(.5)
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
        private double CENTER_POS;
        private final double INCREMENT;
        public Rotator(HardwareMap hardwareMap, String id, double minPos, double maxPos, double centerPos, double increment) {
          super(hardwareMap, id, minPos, maxPos);
          CENTER_POS = centerPos;
          INCREMENT = increment;
        }
        public void center() {
          setPosition(CENTER_POS);
        }
        public void setCenterPos() {
          CENTER_POS = getPosition();
        }
        @Override public double getIncrement() { return INCREMENT; }
    }
    public static class Releaser extends CustomServo {
        public enum State {
            CLOSED, OPEN
        }
        public Releaser(HardwareMap hardwareMap, String id, double minPos, double maxPos) {
            super(hardwareMap, id, minPos, maxPos);
        }
        public void open() {
            goToMinPos();
        }
        public void close() {
            goToMaxPos();
        }

        public State getState() {
            double pos = getPosition();
            final double ERROR = .01;
            if(Math.abs(pos - getMinPos()) < ERROR)
                return State.OPEN;
            else if(Math.abs(pos - getMaxPos()) < ERROR)
                return State.CLOSED;
            else return null;
        }
    }
}
