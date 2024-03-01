package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
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
    private final CustomServo[] servos;
    private final ScheduledExecutorService executorService;
    public Outtake(HardwareMap hardwareMap, String motorName, String flipperName, String extenderName, String armRotatorName, String pixelRotatorName, String releaserName) {
        motor = new Motor(hardwareMap, motorName);
        flipper = new Flipper(hardwareMap, flipperName, .57, .78);
        extender = new CustomServo(hardwareMap, extenderName, .37, 1);
        armRotator = new Rotator(hardwareMap, armRotatorName, 0, 1, .44, .005);
        pixelRotator = new Rotator(hardwareMap, pixelRotatorName, 0, 1, .664, .008);
        releaser = new Releaser(hardwareMap, releaserName, .5, .8);
        //flipper.setPosition(flipper.getMinPos());
        //extender.setPosition(extender.getMinPos());
//        center();
        servos = new CustomServo[]{flipper, extender, armRotator, pixelRotator, releaser};
        executorService = Executors.newSingleThreadScheduledExecutor();
    }
    public void raise() {
        flipper.goToMaxPos();
        executorService.schedule(extender::goToMaxPos, 1400, TimeUnit.MILLISECONDS);
//        center();
    }
    public Action lower() {
//        center();
        extender.goToMinPos();
        return flipper.unflip(.7);
    }
    public void center() {
        armRotator.center();
        pixelRotator.center();
    }
    public void release() {
        releaser.open();
        executorService.schedule(releaser::close, 1000, TimeUnit.MILLISECONDS);
    }

    public void update() {
        for(CustomServo servo : servos)
            servo.update();
    }

    public void moveRight() {
        armRotator.rotateIncrementally();
        pixelRotator.rotateIncrementally();
    }
    public void moveLeft() {
        armRotator.unrotateIncrementally();
        pixelRotator.unrotateIncrementally();
    }

    private class Rotator extends CustomServo {
        private double CENTER_POS;
        private final double INCREMENT;
        public Rotator(String name, HardwareMap hardwareMap, String id, double minPos, double maxPos, double centerPos, double increment) {
          super(name, hardwareMap, id, minPos, maxPos);
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
    private class Releaser extends CustomServo {
        public Releaser(HardwareMap hardwareMap, String id, double minPos, double maxPos) {
            super(hardwareMap, id, minPos, maxPos);
        }
        public void open() {
            goToMinPos();
        }
        public void close() {
            goToMaxPos();
        }
    }
}
