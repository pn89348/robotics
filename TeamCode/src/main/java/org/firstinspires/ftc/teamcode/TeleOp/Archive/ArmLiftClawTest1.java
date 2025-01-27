package org.firstinspires.ftc.teamcode.TeleOp.Archive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name = "V1 Arm PID W/ Lifts and Claw", group = "Linear OpMode")
public class ArmLiftClawTest1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double oldTime = 0;
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;

    //    private DcMotorEx liftL = null;
    private DcMotorEx liftR = null;
    private DcMotorEx arm = null;
    private Servo claw = null;
    private boolean liftRunningToPosition = false;
    double maxPower = 1;
    double powerChangeSensitivity = 0.25;
    public static int maxLiftHeight = 3200;

    // arm pid variables
    private static final double ENCODER_TICKS_PER_REV = 1993.6;
    // These values need to be determined during calibration:
    private static final int ENCODER_VALUE_AT_90_DEGREES = -647; // Change this to actual encoder value when arm is vertical
    private static final int MIN_ENCODER_VALUE = 0; // Change to encoder value at -45 degrees
    private static final int MAX_ENCODER_VALUE = 1300; // Change to encoder value at 225 degrees
    public static double kP = 0.004;  // Tune
    public static double kI = 0.0;
    public static double kD = 0.0005;
    public static double kG = 0.15; // Tune
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private double lastTime = 0;
    private double armTargetPosition = 1200;
    private boolean armRunningToPosition = false;

    private double getArmAngleInRadians() {
        // Convert current encoder position to angle relative to vertical (90 degrees)
        double encoderDifference = arm.getCurrentPosition() - ENCODER_VALUE_AT_90_DEGREES;
        return -(encoderDifference / ENCODER_TICKS_PER_REV) * 2 * Math.PI;
    }

    private double calculateArmPID(double targetPosition) {
        double currentTime = pidTimer.seconds();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        double currentPosition = arm.getCurrentPosition();
        double error = targetPosition - currentPosition;

        // Limit the integral windup
        if (Math.abs(error) < 100) {
            integralSum += error * deltaTime;
        } else {
            integralSum = 0;
        }

        double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0;
        lastError = error;

        // Calculate gravity compensation based on arm angle from vertical
        double angleFromVertical = getArmAngleInRadians();
        double gravityComp = kG * Math.sin(angleFromVertical); // Use sin because angle is relative to vertical

        double output = (error * kP) + (integralSum * kI) + (derivative * kD) + gravityComp;
        return Math.min(Math.max(output, -1), 1);
    }


    @Override
    public void runOpMode() {
//        liftL = hardwareMap.get(DcMotorEx.class, "lift_left");
//        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
//        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftR = hardwareMap.get(DcMotorEx.class, "lift_right");
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD); // Adjust as needed
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "claw");

//        double liftLPos = liftL.getCurrentPosition();
        double liftRPos = liftR.getCurrentPosition();
        double armPos = arm.getCurrentPosition();
        double clawPos = claw.getPosition();


//        telemetry.addData("Left Lift Position:", liftLPos);
        telemetry.addData("Right Lift Position:", liftRPos);
        telemetry.addData("Arm Position:", armPos);
        telemetry.addData("Claw Position:", clawPos);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        pidTimer.reset();

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

//            liftLPos = liftL.getCurrentPosition();
            liftRPos = liftR.getCurrentPosition();
            armPos = arm.getCurrentPosition();
            clawPos = claw.getPosition();

            if (gamepad1.dpad_left) {
//                liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftRunningToPosition = false;
            } else if (gamepad1.dpad_up) {
//                liftL.setTargetPosition(maxLiftHeight);
//                liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftL.setPower(maxPower);
                liftR.setTargetPosition(maxLiftHeight);
                liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftR.setPower(maxPower);
                liftRunningToPosition = true;
            } else if (gamepad1.dpad_down) {
//                liftL.setTargetPosition(0);
//                liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftL.setPower(maxPower);
                liftR.setTargetPosition(0);
                liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftR.setPower(maxPower);
                liftRunningToPosition = true;
            } else if (gamepad1.dpad_right) {
                // Emergency stop
//                liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                liftL.setPower(0);
                liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftR.setPower(0);
                liftRunningToPosition = false;
            }

            // Check if RUN_TO_POSITION has completed
            if (liftRunningToPosition && !liftR.isBusy()) { // && !liftL.isBusy()
                // Motor has reached its target
                liftRunningToPosition = false;
//                liftL.setPower(0); // Stop motor
                liftR.setPower(0);
//                liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (!liftRunningToPosition) {
                double trigger = gamepad1.right_trigger - gamepad1.left_trigger; // number between -1 and 1

                maxPower = Math.max(0, Math.min(maxPower, 1));
                double powerApplied = trigger * maxPower;
//                liftL.setPower(powerApplied);
                liftR.setPower(powerApplied);
                telemetry.addData("Triggers:", trigger);
                telemetry.addData("Current Power:", powerApplied);
                telemetry.addData("Max Power Multiplier", maxPower);
            }

            // Arm control
            if (gamepad1.y) { // Example preset position
                armTargetPosition = 900; // Adjust value
                armRunningToPosition = true;
            } else if (gamepad1.a) {
                armTargetPosition = 1300;
                armRunningToPosition = true;
            }

            if (armRunningToPosition) {
                // Constrain target position to valid range
                armTargetPosition = Math.min(Math.max(armTargetPosition, MIN_ENCODER_VALUE), MAX_ENCODER_VALUE);
                double power = calculateArmPID(armTargetPosition);
                arm.setPower(power);
            }
            double leftStickY = -gamepad1.left_stick_y;
            if (Math.abs(leftStickY) > 0.1) {
                armRunningToPosition = false;
                double angleFromVertical = getArmAngleInRadians();
                double gravityComp = kG * Math.sin(angleFromVertical);
                arm.setPower(leftStickY * maxPower + gravityComp);
            } else {
                if (!armRunningToPosition) {
                    armTargetPosition = arm.getCurrentPosition();
                }
                armRunningToPosition = true;
                double power = calculateArmPID(armTargetPosition);
                arm.setPower(power);
            }

            double rightStickY = -gamepad1.right_stick_y;
            double newClawPos = claw.getPosition() + rightStickY / 50;
            newClawPos = Math.min(1.0, Math.max(0.0, newClawPos));
            claw.setPosition(newClawPos);

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
//            telemetry.addData("Left Lift Position:", liftL.getCurrentPosition());
            telemetry.addData("Right Lift Position:", liftR.getCurrentPosition());
            telemetry.addData("(Right) Lift Velocity:", liftR.getVelocity());
//            telemetry.addData("Lifts Encoder Difference:", liftR.getCurrentPosition() - liftL.getCurrentPosition());
            telemetry.addData("Is Lift Running to Position:", liftRunningToPosition);
            telemetry.addData("Is (Right) Lift Motor Busy:", liftR.isBusy());
            telemetry.addData("", "");
            telemetry.addData("Arm Position:", arm.getCurrentPosition());
            telemetry.addData("Arm Angle (deg):", Math.toDegrees(getArmAngleInRadians()));
            telemetry.addData("90 Degree Reference (Encoder):", ENCODER_VALUE_AT_90_DEGREES);
            telemetry.addData("Arm Velocity (Encoder):", arm.getVelocity());
            telemetry.addData("Arm Angular Velocity (deg/s):", Math.toDegrees(arm.getVelocity() / ENCODER_TICKS_PER_REV * 2 * Math.PI));
            telemetry.addData("Arm Target Position:", armTargetPosition);
            telemetry.addData("Is Arm Running to Position:", armRunningToPosition);
            telemetry.addData("", "");
            telemetry.addData("Claw Position:", claw.getPosition());
//            telemetry.addData("Lift Motor Busy:", liftL.isBusy() || liftR.isBusy());
            telemetry.addData("", "");
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
