package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Dual Lift Test", group="Component Testing")
public class DualLiftTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lift1 = null;
    private DcMotor lift2 = null;
    int MAX_HEIGHT = 3100;
    double oldTime = 0;

    double maxPowerMult = 0.5; // Set to 1 for full power
    double powerChangeSensitivity = 0.25;

    @Override
    public void runOpMode() {
        lift1 = hardwareMap.get(DcMotor.class, "lift_right");
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift2 = hardwareMap.get(DcMotor.class, "lift_left");
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double position1 = lift1.getCurrentPosition();
        double position2 = lift2.getCurrentPosition();


        boolean prevRightBumper = false;
        boolean prevLeftBumper = false;


        telemetry.addData("Encoder Position:", position1);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        boolean isRunningToPosition = false;

        while (opModeIsActive()) {
            if (gamepad1.x) {
                lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                isRunningToPosition = false;
            } else if (gamepad1.y) {
                lift1.setTargetPosition(MAX_HEIGHT);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setTargetPosition(MAX_HEIGHT);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setPower(1);
                isRunningToPosition = true;
            } else if (gamepad1.a) {
                lift1.setTargetPosition(0);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setTargetPosition(0);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setPower(1);
                isRunningToPosition = true;
            } else if (gamepad1.b) {
                // Emergency stop
                lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift1.setPower(0);
                lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift2.setPower(0);
                isRunningToPosition = false;
            }

            // Check if RUN_TO_POSITION has completed
            if (isRunningToPosition && !lift1.isBusy() && !lift2.isBusy()) {
                // Motor has reached its target
                isRunningToPosition = false;
                lift1.setPower(0); // Stop motor
                lift2.setPower(0);
                lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;
            double oldPos1 = position1;
            position1 = lift1.getCurrentPosition();
            position2 = lift2.getCurrentPosition();
            double posDelta = position2 - position1;
            double velocity1 = (position1 - oldPos1) / loopTime;
            telemetry.addData("Encoder Position:", position1);
            telemetry.addData("Encoder Velocity:", velocity1);
            telemetry.addData("Is Running to Position:", isRunningToPosition);
            telemetry.addData("Motor Busy:", lift1.isBusy());
            // Allow manual control only if not running to a position
            if (!isRunningToPosition) {
                double trigger = gamepad1.right_trigger - gamepad1.left_trigger; // number between -1 and 1
                boolean rightBumper = gamepad1.right_bumper;
                boolean leftBumper = gamepad1.left_bumper;
                if (prevRightBumper && !rightBumper) { // on release of button
                    maxPowerMult += powerChangeSensitivity;
                }
                if (prevLeftBumper && !leftBumper) {
                    maxPowerMult -= powerChangeSensitivity;
                }
                prevRightBumper = rightBumper;
                prevLeftBumper = leftBumper;

                maxPowerMult = Math.max(0, Math.min(maxPowerMult, 1));
                double powerApplied = trigger * maxPowerMult;
                lift1.setPower(powerApplied);
                lift2.setPower(powerApplied);
                telemetry.addData("Triggers:", trigger);
                telemetry.addData("Current Power:", powerApplied);
                telemetry.addData("Max Power Multiplier", maxPowerMult);
            }
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
