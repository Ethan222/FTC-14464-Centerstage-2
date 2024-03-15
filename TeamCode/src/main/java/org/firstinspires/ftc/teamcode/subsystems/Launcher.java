package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher extends CustomServo {
    private static final double INCREMENT = .005;
    public Launcher(HardwareMap hardwareMap, String id) {
        super(hardwareMap, id, .18, 1);
    }
    public void launch() { rotateBy(-INCREMENT); }
    public void reset() {
        rotateBy(INCREMENT);
    }
}
