package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher extends CustomServo {
    private static final double INCREMENT = .004;
    public Launcher(HardwareMap hardwareMap, String id) {
        super(hardwareMap, id, .18, 1);
    }
    public void launch() { rotateBy(-INCREMENT); }
    public void reset() {
        rotateBy(INCREMENT);
    }
    public void launch(double val) {
        rotateBy(-val * INCREMENT);
    }
    public void reset(double val) {
        rotateBy(val * INCREMENT);
    }
}
