package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class Outtake {
    public Motor motor;
    public Flipper flipper;
    public CustomServo extender, releaser;
    public Rotator armRotator, pixelRotator;
    private final CustomServo[] servos;
    private final ScheduledExecutorService executorService;
    public Outtake(HardwareMap hardwareMap, String motorName, String flipperName, String extenderName, String armRotatorName, String pixelRotatorName, String releaserName) {
        motor = new Motor(hardwareMap, motorName);
        flipper = new Flipper("flipper", hardwareMap, flipperName, .57, .78);
        extender = new CustomServo("extender", hardwareMap, extenderName, .37, 1);
        armRotator = new Rotator("armRotator", hardwareMap, armRotatorName, 0, 1, .44, .005);
        pixelRotator = new Rotator("pixelRotator", hardwareMap, pixelRotatorName, 0, 1, .664, .008);
        releaser = new CustomServo("releaser", hardwareMap, releaserName, .5, .8);
        flipper.setPosition(flipper.getMinPos());
        extender.setPosition(extender.getMinPos());
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
        releaser.goToMinPos();
        executorService.schedule(releaser::goToMaxPos, 1000, TimeUnit.MILLISECONDS);
    }

    public void update() {
        for(CustomServo servo : servos)
            servo.update();
    }

    public void moveRight() {
        armRotator.rotate();
        pixelRotator.rotate();
    }
    public void moveLeft() {
        armRotator.unrotate();
        pixelRotator.unrotate();
    }
}
