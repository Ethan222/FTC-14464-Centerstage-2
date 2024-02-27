package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    public ServoEx[] servos;
    public Outtake(HardwareMap hardwareMap, String[] servoNames) {
        servos = new ServoEx[servoNames.length];
        for(int i = 0; i < servoNames.length; i++) {
            servos[i] = new SimpleServo(hardwareMap, servoNames[i], 0, 180);
        }
        servos[0].setRange(0, .43);
    }
    public String getTelemetry() {
        StringBuilder ret = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            ret.append(String.format("Servo %d: %.2f / %.2f deg \n", i, servos[i].getPosition(), servos[i].getAngle()));
        }
        return ret.toString();
    }
}
