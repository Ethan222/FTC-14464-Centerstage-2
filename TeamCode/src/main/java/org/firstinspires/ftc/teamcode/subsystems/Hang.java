package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hang extends Motor {
    public final CustomServo leftServo, rightServo;
    public Hang(HardwareMap hardwareMap, String motorName, String servo1Name, String servo2Name) {
        super(hardwareMap, motorName);
        leftServo = new CustomServo(hardwareMap, servo1Name);
        rightServo = new CustomServo(hardwareMap, servo2Name);
    }
    public void raise() {
        leftServo.setPosition(.77);
        rightServo.setPosition(.65);
    }
    public void lower() {
        leftServo.setPosition(.4);
        rightServo.setPosition(.34);
    }
}
