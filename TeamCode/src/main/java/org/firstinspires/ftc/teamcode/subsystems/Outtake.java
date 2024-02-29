package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class Outtake {
    public CustomServo flipper, extender, armRotator, pixelRotator, releaser;
    private final CustomServo[] servos;
    private final ScheduledExecutorService scheduler;
    public Outtake(HardwareMap hardwareMap, String flipperName, String extenderName, String armRotatorName, String pixelRotatorName, String releaserName) {
        flipper = new CustomServo("flipper", hardwareMap, flipperName, .59, .78);
        extender = new CustomServo("extender", hardwareMap, extenderName, .15, .69);
        armRotator = new Rotator("armRotator", hardwareMap, armRotatorName, .33, .64, .48);
        pixelRotator = new Rotator("pixelRotator", hardwareMap, pixelRotatorName, 0, 1, .68);
        releaser = new CustomServo("releaser", hardwareMap, releaserName, .1, .9);
        flipper.setPosition(flipper.getMinPos());
        extender.setPosition(extender.getMinPos());
        armRotator.setPosition(.48);
        pixelRotator.setPosition(.68);
        servos = new CustomServo[]{flipper, extender, armRotator, pixelRotator, releaser};
        scheduler = Executors.newSingleThreadExecutor();
    }
    public void raise() {
        flipper.goToMaxPosition();
        scheduler.schedule(extender::goToMaxPosition, 250, TimeUnit.MILLISECONDS);
    }
    public void lower() {
        center();
        extender.goToMinPosition();
        flipper.goToMinPosition();
    }
    public void center() {
        armRotator.center();
        pixelRotator.center();
    }
    public void release() {
        releaser.goToMinPosition();
        scheduler.schedule(releaser::goToMaxPosition, 1000, TimeUnit.MILLISECONDS);
    }
    public String getTelemetry() {
        StringBuilder ret = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            ret.append("\n").append(servos[i].getTelemetry());
        }
        return ret.toString();
    }

    public void update() {
        for(CustomServo servo : servos)
            servo.update();
    }
}
