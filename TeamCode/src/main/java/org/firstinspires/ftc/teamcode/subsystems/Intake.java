package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private final Motor motor;
    private Action action;
    public Intake(HardwareMap hardwareMap, String motorName) {
        motor = new Motor(hardwareMap, motorName, true);
    }
    public void in(double power) {
        motor.setPower(power);
    }
    public void in() { in(1); }
    public void out(double power) {
        motor.setPower(-power);
    }
    public void out() { out(1); }
    public void stop() {
        motor.stop();
        if(action != null && action.getClass().equals(InWithPeriodicOut.class)) cancel();
    }
    public double getPower() {
        return motor.getPower();
    }
    public String getTelemetry() {
        return String.format("%.1f", motor.getPower());
    }

    public class InWithPeriodicOut implements Action {
        public static final String INTAKING = "INTAKING", OUTTAKING = "OUTTAKING";
        public static final double INTAKE_TIME = 1.5, OUTTAKE_TIME = 0.2;
        private boolean initialized = false;
        private ElapsedTime timer;
        private String state;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                timer = new ElapsedTime();
                state = INTAKING;
                initialized = true;
                return true;
            }
            if(state.equals(INTAKING)) {
                in();
                if(timer.seconds() > INTAKE_TIME) {
                    state = OUTTAKING;
                    timer.reset();
                }
            } else {
                out();
                if(timer.seconds() > OUTTAKE_TIME) {
                    state = INTAKING;
                    timer.reset();
                }
            }
            return true;
        }
    }
    public Action inWithPeriodicOut() {
        action = new InWithPeriodicOut();
        return action;
    }
    public void cancel() {
        action = new NullAction();
    }
}