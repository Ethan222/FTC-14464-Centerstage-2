package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Flipper extends CustomServo {
    public static final String DOWN = "DOWN", UP = "UP";
    public static double UP_POSITION = .87, DOWN_POSITION = .50, SLOW_DOWN_POSITION = 0.63, INCREMENT = .001;
    public static double MAX_SPEED = .02, MIN_SPEED = 1e-5, GROWTH_CONSTANT = 75;
    private final double MIDPOINT = (UP_POSITION + DOWN_POSITION) / 2;
    public Flipper(HardwareMap hardwareMap, String id, double minPos, double maxPos) {
        super(hardwareMap, id, minPos, maxPos);
        setPosition(DOWN_POSITION);
    }
    public class Unflip implements Action {
        private double logisticEquation(double L, double k, double x0, double x) {
            return L / (1 + Math.exp(-k * (x - x0)));
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double psn = getPosition();
            double speed = Math.max(logisticEquation(MAX_SPEED, GROWTH_CONSTANT, MIDPOINT, psn), MIN_SPEED);
            packet.put("psn", psn);
            packet.put("speed", speed);
            if(psn > DOWN_POSITION + .05) {
                rotateBy(-speed);
                return true;
            }
            setPosition(DOWN_POSITION);
            return false;
        }
    }
    public Action unflip(double delay) {
        return new SequentialAction(
                new SleepAction(delay),
                new Unflip()
        );
    }
    public Action unflip() { return new Unflip(); }
    @Override public double getIncrement() {
        return INCREMENT;
    }

    public Action flip() {
        return goToPos(UP_POSITION, .001);
    }

    public String getState() {
        double psn = getPosition();
        final double ERROR = .03;
        if(Math.abs(psn - DOWN_POSITION) < ERROR)
            return DOWN;
        else if(Math.abs(psn - UP_POSITION) < ERROR)
            return UP;
        else return "";
    }
}
