package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Controller Test", group="Testing and Tuning")
public class ControllerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Display initialization message
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Gamepad1
            telemetry.addData("Gamepad1 - Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Gamepad1 - Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Gamepad1 - Right Stick X", gamepad1.right_stick_x);
            telemetry.addData("Gamepad1 - Right Stick Y", gamepad1.right_stick_y);

            telemetry.addData("Gamepad1 - Left Trigger", gamepad1.left_trigger);
            telemetry.addData("Gamepad1 - Right Trigger", gamepad1.right_trigger);

            telemetry.addData("Gamepad1 - A Button", gamepad1.a);
            telemetry.addData("Gamepad1 - B Button", gamepad1.b);
            telemetry.addData("Gamepad1 - X Button", gamepad1.x);
            telemetry.addData("Gamepad1 - Y Button", gamepad1.y);
            telemetry.addData("Gamepad1 - Left Bumper", gamepad1.left_bumper);
            telemetry.addData("Gamepad1 - Right Bumper", gamepad1.right_bumper);

            telemetry.addData("Gamepad1 - DPad Up", gamepad1.dpad_up);
            telemetry.addData("Gamepad1 - DPad Down", gamepad1.dpad_down);
            telemetry.addData("Gamepad1 - DPad Left", gamepad1.dpad_left);
            telemetry.addData("Gamepad1 - DPad Right", gamepad1.dpad_right);

            // Gamepad2
            telemetry.addData("Gamepad2 - Left Stick X", gamepad2.left_stick_x);
            telemetry.addData("Gamepad2 - Left Stick Y", gamepad2.left_stick_y);
            telemetry.addData("Gamepad2 - Right Stick X", gamepad2.right_stick_x);
            telemetry.addData("Gamepad2 - Right Stick Y", gamepad2.right_stick_y);

            telemetry.addData("Gamepad2 - Left Trigger", gamepad2.left_trigger);
            telemetry.addData("Gamepad2 - Right Trigger", gamepad2.right_trigger);

            telemetry.addData("Gamepad2 - A Button", gamepad2.a);
            telemetry.addData("Gamepad2 - B Button", gamepad2.b);
            telemetry.addData("Gamepad2 - X Button", gamepad2.x);
            telemetry.addData("Gamepad2 - Y Button", gamepad2.y);
            telemetry.addData("Gamepad2 - Left Bumper", gamepad2.left_bumper);
            telemetry.addData("Gamepad2 - Right Bumper", gamepad2.right_bumper);

            telemetry.addData("Gamepad2 - DPad Up", gamepad2.dpad_up);
            telemetry.addData("Gamepad2 - DPad Down", gamepad2.dpad_down);
            telemetry.addData("Gamepad2 - DPad Left", gamepad2.dpad_left);
            telemetry.addData("Gamepad2 - DPad Right", gamepad2.dpad_right);

            // Update the displayed data
            telemetry.update();
        }
    }
}
