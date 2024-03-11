package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderMotor extends Motor {
    private final int[] presets;
    public EncoderMotor(HardwareMap hardwareMap, String name, int[] psns) {
        super(hardwareMap, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        presets = psns;
    }

    public EncoderMotor(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, new int[]{0});
    }

    public Action goToPosition(int targetPos, double power, double timeoutTime) {
        return new Action() {
            private boolean initialized = false;
            private double previousPos;
            private ElapsedTime timer;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double pos = motor.getCurrentPosition();
                packet.put("pos", pos);
                if(Math.abs(pos - targetPos) < 5) {
                    motor.setPower(0);
                    return false;
                } else if(!initialized) {
                    motor.setPower(pos > targetPos ? power : -power);
                    timer = new ElapsedTime();
                    initialized = true;
                } else if(pos != previousPos)
                    timer.reset();
                else if(timer.seconds() > timeoutTime) {
                    motor.setPower(0);
                    return false;
                }
                previousPos = pos;
                return true;
            }
        };
    }
    public Action goToPosition(int pos, double power) {
        return goToPosition(pos, power, 3);
    }
    public Action goToPosition(int pos) { return goToPosition(pos, 1); }
    public Action goToPreset(int preset, double power) {
        return goToPosition(presets[preset], power);
    }
    public Action goToPreset(int preset) { return goToPreset(preset, 1); }
    public int getPosition() {
        return motor.getCurrentPosition();
    }
    public void setPreset(int preset) {
        presets[preset] = getPosition();
    }
    public boolean isBusy() {
        return motor.isBusy();
    }
    public int getCurrentPreset() {
        int pos = getPosition();
        for(int i = 0; i < presets.length; i++) {
            if(Math.abs(pos - presets[i]) < 5)
                return i;
        }
        return -1;
    }
    @Override public String getTelemetry() {
        int currentPreset = getCurrentPreset();
        String presetString = (currentPreset == -1) ? "" : ("(preset " + currentPreset + ")");
        return String.format("%d %s [%.1f]", getPosition(), presetString, motor.getPower());
    }
}