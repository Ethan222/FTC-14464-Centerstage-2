package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PixelPusher {
    private ServoEx servo;
    private final ElapsedTime timer;
    public static double outAndInTime = .3;
    public PixelPusher(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.get(ServoEx.class, name);
        timer = new ElapsedTime();
    }

    public Action out(double deg) {
        return telemetryPacket -> {
            servo.rotateByAngle(deg);
            return false;
        };
    }
    public Action in(double deg) {
        return telemetryPacket -> {
            servo.rotateByAngle(-deg);
            return false;
        };
    }
    public Action out() {
        return telemetryPacket -> {
            servo.setPosition(1);
            return false;
        };
    }
    public Action in() {
        return telemetryPacket -> {
            servo.setPosition(0);
            return false;
        };
    }
    public Action outAndIn() {
        return new SequentialAction(
                out(),
                new SleepAction(outAndInTime),
                in()
        );
    }

    public String getState() {
        double psn = servo.getPosition();
        if(psn < .02)
            return "IN";
        else if(psn > .98)
            return "OUT";
        else return "PARTLY OUT";
    }
    public String getTelemetry() {
        return String.format("%s (%.2f / %.2f deg)", getState(), servo.getPosition(), servo.getAngle());
    }
}
