package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Test", group="Component Testing")
public class ServoTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo servo1 = null;
    double oldTime = 0;

    @Override
    public void runOpMode() {

        servo1 = hardwareMap.get(Servo.class, "servo1");

        boolean prevRightBumper = false;
        boolean prevLeftBumper = false;

        double currentPos1 = servo1.getPosition();
        telemetry.addData("Servo 1 Pos:", servo1.getPosition());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // make the servo position increase when right trigger is pressed and decrease when left trigger is pressed, based on how far the trigger is pressed
            double trigger = gamepad1.right_trigger - gamepad1.left_trigger;
            double newPos = currentPos1 + trigger/200;
            if (newPos > 1) {
                newPos = 1;
            } else if (newPos < 0) {
                newPos = 0;
            }
            servo1.setPosition(newPos);

            if (gamepad1.y) {
                servo1.setPosition(0.5);
            } else if (gamepad1.b) {
                servo1.setPosition(0.0);
            } else if (gamepad1.x) {
                servo1.setPosition(1.0);
            }
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;
            double oldPos1 = currentPos1;
            currentPos1 = servo1.getPosition();
            double velocity = (currentPos1 - oldPos1) / loopTime;
            telemetry.addData("Servo 1 Pos:", currentPos1);
            telemetry.addData("Servo 1 Vel:", velocity);
            telemetry.addData("Triggers:", trigger);
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }
}
