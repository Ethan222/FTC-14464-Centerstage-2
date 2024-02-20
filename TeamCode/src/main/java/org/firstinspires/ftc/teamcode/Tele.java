package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Tele extends LinearOpMode {
    private GamepadEx driver1;

    @Override public void runOpMode() throws InterruptedException {
        driver1 = new GamepadEx(gamepad1);
        ToggleButtonReader toggleButtonReader = new ToggleButtonReader(driver1, GamepadKeys.Button.A);
    }
}
