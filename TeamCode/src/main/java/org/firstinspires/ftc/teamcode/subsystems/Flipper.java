package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Flipper extends CustomServo {
    public static final String DOWN = "DOWN", UP = "UP";
    public static double UP_POSITION = .87, NORMAL_SPEED = .01, SLOW_SPEED = .0006, SLOW_DOWN_POSITION = 0.63+.005, INCREMENT = .001;
    public Flipper(HardwareMap hardwareMap, String id, double minPos, double maxPos) {
        super(hardwareMap, id, minPos, maxPos);
    }
    public class Unflip implements Action {
        private boolean canceled;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(canceled) return false;
            if(getPosition() > SLOW_DOWN_POSITION) {
                rotateBy(-NORMAL_SPEED);
                return true;
            } else if(getPosition() > getMinPos()) {
                rotateBy(-SLOW_SPEED);
                return true;
            } else return false;
        }
        public void cancel() {
            canceled = true;
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
        return goToPos(UP_POSITION);
    }

    public String getState() {
        double psn = getPosition();
        final double ERROR = .03;
        if(Math.abs(psn - getMinPos()) < ERROR)
            return DOWN;
        else if(Math.abs(psn - UP_POSITION) < ERROR)
            return UP;
        else return "";
    }
}
