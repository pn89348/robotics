package org.firstinspires.ftc.teamcode.TeleOp.Archive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name="Lifts and 3 Servos Test", group="Linear OpMode")
public class LiftsServosTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor liftL = null;
    private DcMotor liftR = null;
    private Servo armL = null;
    private Servo armR = null;
    private Servo claw = null;
    int maxLiftHeight = 3200;
    double oldTime = 0;
    double maxPower = 1;
    double powerChangeSensitivity = 0.25;

    @Override
    public void runOpMode() {
        liftL = hardwareMap.get(DcMotor.class, "lift_left");
        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftR = hardwareMap.get(DcMotor.class, "lift_right");
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armL = hardwareMap.get(Servo.class, "arm_left");
        armR = hardwareMap.get(Servo.class, "arm_right");
        claw = hardwareMap.get(Servo.class, "claw");

        double liftLPos = liftL.getCurrentPosition();
        double liftRPos = liftR.getCurrentPosition();
        double armLPos = armL.getPosition();
        double armRPos = armR.getPosition();
        double clawPos = claw.getPosition();

        boolean prevRightBumper = false;
        boolean prevLeftBumper = false;

        boolean liftRunningToPosition = false;

        telemetry.addData("Left Lift Position:", liftLPos);
        telemetry.addData("Right Lift Position:", liftRPos);
        telemetry.addData("Left Arm Position:", armLPos);
        telemetry.addData("Right Arm Position:", armRPos);
        telemetry.addData("Claw Position:", clawPos);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;
            if (prevRightBumper && !rightBumper) { // on release of button
                maxPower += powerChangeSensitivity;
            }
            if (prevLeftBumper && !leftBumper) {
                maxPower -= powerChangeSensitivity;
            }
            prevRightBumper = rightBumper;
            prevLeftBumper = leftBumper;

            liftLPos = liftL.getCurrentPosition();
            liftRPos = liftR.getCurrentPosition();
            armLPos = armL.getPosition();
            armRPos = armR.getPosition();
            clawPos = claw.getPosition();

            if (gamepad1.dpad_left) {
                liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftRunningToPosition = false;
            } else if (gamepad1.dpad_up) {
                liftL.setTargetPosition(maxLiftHeight);
                liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftL.setPower(maxPower);
                liftR.setTargetPosition(maxLiftHeight);
                liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftR.setPower(maxPower);
                liftRunningToPosition = true;
            } else if (gamepad1.dpad_down) {
                liftL.setTargetPosition(0);
                liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftL.setPower(maxPower);
                liftR.setTargetPosition(0);
                liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftR.setPower(maxPower);
                liftRunningToPosition = true;
            } else if (gamepad1.dpad_right) {
                // Emergency stop
                liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftL.setPower(0);
                liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftR.setPower(0);
                liftRunningToPosition = false;
            }

            // Check if RUN_TO_POSITION has completed
            if (liftRunningToPosition && !liftL.isBusy() && !liftR.isBusy()) {
                // Motor has reached its target
                liftRunningToPosition = false;
                liftL.setPower(0); // Stop motor
                liftR.setPower(0);
                liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (!liftRunningToPosition) {
                double trigger = gamepad1.right_trigger - gamepad1.left_trigger; // number between -1 and 1

                maxPower = Math.max(-1, Math.min(maxPower, 1));
                double powerApplied = trigger * maxPower;
                liftL.setPower(powerApplied);
                liftR.setPower(powerApplied);
                telemetry.addData("Triggers:", trigger);
                telemetry.addData("Current Power:", powerApplied);
                telemetry.addData("Max Power Multiplier", maxPower);
            }

            double leftStickY = -gamepad1.left_stick_y;
            double newArmPos = armL.getPosition() + leftStickY/50;
            newArmPos = Math.min(1.0, Math.max(0.0, newArmPos));
            armL.setPosition(newArmPos);
            armR.setPosition(newArmPos);

            double rightStickY = -gamepad1.right_stick_y;
            double newClawPos = claw.getPosition() + rightStickY/50;
            newClawPos = Math.min(1.0, Math.max(0.0, newClawPos));
            claw.setPosition(newClawPos);

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            telemetry.addData("Left Lift Position:", liftL.getCurrentPosition());
            telemetry.addData("Right Lift Position:", liftR.getCurrentPosition());
            telemetry.addData("Lifts Encoder Difference:", liftR.getCurrentPosition() - liftL.getCurrentPosition());
            telemetry.addData("Left Arm Position:", armL.getPosition());
            telemetry.addData("Right Arm Position:", armR.getPosition());
            telemetry.addData("Claw Position:", claw.getPosition());
            telemetry.addData("Is Lift Running to Position:", liftRunningToPosition);
            telemetry.addData("Lift Motor Busy:", liftL.isBusy() || liftR.isBusy());
            // Allow manual control only if not running to a position

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
