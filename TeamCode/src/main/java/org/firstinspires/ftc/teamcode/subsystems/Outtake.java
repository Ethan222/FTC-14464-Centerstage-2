package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    public CustomServo[] servos;
    public Outtake(HardwareMap hardwareMap, String[] servoNames) {
        servos = new CustomServo[servoNames.length];
        servos[0] = new CustomServo(hardwareMap, servoNames[0], .58, .78);
        servos[1] = new CustomServo(hardwareMap, servoNames[1], 0, 1);
        servos[2] = new CustomServo(hardwareMap, servoNames[2], 0, 1);
        servos[3] = new CustomServo(hardwareMap, servoNames[3], .15, .7);
    }
    public String getTelemetry() {
        StringBuilder ret = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            ret.append(String.format("\n servo %d: %.2f", i, servos[i].getPosition()));
        }
        return ret.toString();
    }
}
