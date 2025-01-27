package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "Arm Tuning Test", group = "Testing and Tuning")
public class ArmTuningTest extends LinearOpMode {
    private DcMotorEx arm = null;
    private ElapsedTime pidTimer = new ElapsedTime();
    private double lastTime = 0;

    // PID Constants
    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.0003;
    public static double kG = 0.25;

    private double integralSum = 0;
    private double lastError = 0;
    private double maxPIDPower = 0.5;

    // Encoder values
    private static final int ENCODER_VALUE_AT_90_DEGREES = -647; // Vertical position
    private static final double ENCODER_TICKS_PER_REV = 1992.6;

    // Modes
    private boolean poweredOffMode = true;
    private boolean gravityCompensationMode = false;
    private boolean pidTestMode = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {
            // Mode selection using gamepad buttons
            if (gamepad1.x) {
                poweredOffMode = true;
                gravityCompensationMode = false;
                pidTestMode = false;
            } else if (gamepad1.y) {
                poweredOffMode = false;
                gravityCompensationMode = true;
                pidTestMode = false;
            } else if (gamepad1.b) {
                poweredOffMode = false;
                gravityCompensationMode = false;
                pidTestMode = true;
            }

            if (poweredOffMode) {
                telemetry.addData("Mode", "Powered Off (Telemetry Only)");
                double angleFromVertical = getArmAngleInRadians();
                double gravityComp = kG * Math.sin(angleFromVertical);
                telemetry.addData("Arm Angle (deg)", Math.toDegrees(angleFromVertical));
                telemetry.addData("Theoretical Gravity Comp Power", gravityComp);
                arm.setPower(gravityComp);
            } else if (gravityCompensationMode) {
                testGravityCompensation();
            } else if (pidTestMode) {
                testPIDControl();
            }

            // Display PID constants for tuning
            if (gamepad1.dpad_up) kP += 0.0001; // Increase kP
            if (gamepad1.dpad_down) kP -= 0.0001; // Decrease kP
            if (gamepad1.dpad_right) kD += 0.0001; // Increase kD
            if (gamepad1.dpad_left) kD -= 0.0001; // Decrease kD

            telemetry.addData("kP", kP);
            telemetry.addData("kD", kD);
            telemetry.addData("kG", kG);
            telemetry.update();
        }
    }

    private void testGravityCompensation() {
        double angleFromVertical = getArmAngleInRadians();
        double gravityComp = kG * Math.sin(angleFromVertical);
        arm.setPower(gravityComp);

        telemetry.addData("Mode", "Gravity Compensation");
        telemetry.addData("Arm Angle (deg)", Math.toDegrees(angleFromVertical));
        telemetry.addData("Gravity Comp Power", gravityComp);
    }

    private void testPIDControl() {
        int targetPosition = gamepad1.a ? -1000 : gamepad1.b ? -200 : ENCODER_VALUE_AT_90_DEGREES; // Use buttons to set targets

        double currentTime = pidTimer.seconds();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        double currentPosition = arm.getCurrentPosition();
        double error = targetPosition - currentPosition;

        // Integral term
        if (Math.abs(error) < 100) integralSum += error * deltaTime;
        else integralSum = 0;

        // Derivative term
        double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0;
        lastError = error;

        // Gravity compensation
        double angleFromVertical = getArmAngleInRadians();
        double gravityComp = kG * Math.sin(angleFromVertical);

        // PID calculation
        double outputPower =
                Math.min(Math.max((error * kP) + (integralSum * kI) + (derivative * kD) + gravityComp, -maxPIDPower), maxPIDPower);

        arm.setPower(outputPower);

        telemetry.addData("Mode", "PID Test");
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Error", error);
        telemetry.addData("Output Power", outputPower);
    }

    private double getArmAngleInRadians() {
        double encoderDifference = arm.getCurrentPosition() - ENCODER_VALUE_AT_90_DEGREES;
        return -(encoderDifference / ENCODER_TICKS_PER_REV) * 2 * Math.PI;
    }
}
