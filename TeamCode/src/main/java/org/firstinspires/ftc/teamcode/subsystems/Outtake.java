package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    public CustomServo flipper, extender, armRotator, pixelRotator, releaser;
    private final CustomServo[] servos;
    public Outtake(HardwareMap hardwareMap, String flipperName, String extenderName, String armRotatorName, String pixelRotatorName, String releaserName) {
        flipper = new CustomServo("flipper", hardwareMap, flipperName, .59, .78);
        extender = new CustomServo("extender", hardwareMap, extenderName, .15, .69);
        armRotator = new CustomServo("armRotator", hardwareMap, armRotatorName, .33, .64);
        pixelRotator = new CustomServo("pixelRotator", hardwareMap, pixelRotatorName, 0, 1);
        releaser = new CustomServo("releaser", hardwareMap, releaserName, .1, .9);
        flipper.setPosition(flipper.getMinPos());
        extender.setPosition(extender.getMinPos());
        armRotator.setPosition(.48);
        pixelRotator.setPosition(.68);
        servos = new CustomServo[]{flipper, extender, armRotator, pixelRotator, releaser};
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
