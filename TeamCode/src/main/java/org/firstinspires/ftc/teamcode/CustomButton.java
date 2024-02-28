package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

// class to represent a button on a gampad
// tracks how long the button has been held down
public class CustomButton {
    public enum State {
        UP, JUST_DOWN, DOWN, JUST_UP
    }
    private final GamepadEx gamepad;
    private final GamepadKeys.Button button;
    private State state;
    private final ElapsedTime timeSinceDown, timeSinceUp;
    public CustomButton(GamepadEx gamepad, GamepadKeys.Button button) {
        this.gamepad = gamepad;
        this.button = button;
        timeSinceDown = new ElapsedTime();
        timeSinceUp = new ElapsedTime();
        state = State.UP;
    }
    public void update() {
        if(gamepad.isDown(button)) {
            if(state == State.UP || state == State.JUST_UP) {
                state = State.JUST_DOWN;
                timeSinceDown.reset();
            } else state = State.DOWN;
        } else {
            if(state == State.DOWN || state == State.JUST_DOWN) {
                state = State.JUST_UP;
                timeSinceUp.reset();
            } else state = State.UP;
        }
    }
    public State getState() { return state; }
    public double getTimeDown() {   // returns the time since the button was pressed
        return timeSinceDown.seconds();
    }
    public double getTimeUp() {
        return timeSinceUp.seconds();
    }
    public String getName() {
        return button.toString().toLowerCase();
    }
}
