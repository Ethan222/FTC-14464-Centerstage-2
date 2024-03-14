package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hang extends Motor {
    public final CustomServo leftServo, rightServo;
    public Hang(HardwareMap hardwareMap, String motorName, String servo1Name, String servo2Name) {
        super(hardwareMap, motorName);
        leftServo = new CustomServo(hardwareMap, servo1Name, .4, .8);
        rightServo = new CustomServo(hardwareMap, servo2Name, .3, .7);
    }
    public void raise() {
        leftServo.setPosition(.76);
        rightServo.setPosition(.65);
    }
    public void lower() {
        leftServo.setPosition(.44);
        rightServo.setPosition(.34);
    }
}
