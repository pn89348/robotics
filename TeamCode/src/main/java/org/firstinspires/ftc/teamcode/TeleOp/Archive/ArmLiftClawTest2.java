package org.firstinspires.ftc.teamcode.TeleOp.Archive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "V2 Arm PID W/ Lifts and Claw", group = "Testing and Tuning")
public class ArmLiftClawTest2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double oldTime = 0;
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;

    private DcMotorEx liftL = null;
    private DcMotorEx liftR = null;
    private DcMotorEx arm = null;
    private Servo claw = null;
    private boolean liftRunningToPosition = false;
    double maxPower = 0.5;
    double powerChangeSensitivity = 0.25;
    public static int maxLiftHeight = 3100;

    // arm pid variables
    private static final double ENCODER_TICKS_PER_REV = 1992.6;
    // These values need to be determined during calibration:
    private static final int ENCODER_VALUE_AT_90_DEGREES = -647; // Change this to actual encoder value when arm is vertical
    private double angleCorrection = 90.0 / 80.0;
    // Refined PID Constants
    public static double kP = 0.002;  // Conservative starting value
    public static double kI = 0.0;    // Start with no integral term
    public static double kD = 0.0005; // Conservative derivative
    public static double kG = 0.12;   // Gravity compensation
    public static double k2G = 0.19; // Gravity comp for PID function, kinetic kG

    // Safe encoder range
    private static final int MIN_ENCODER_VALUE = -100; // Vertical position
    private static final int MAX_ENCODER_VALUE = -1200; // Testing upper limit
    private double integralSum = 0;
    private double lastError = 0;
    private double maxPIDPower = 0.5;
    private ElapsedTime pidTimer = new ElapsedTime();
    private double lastTime = 0;
    private double armTargetPosition = -200;
    private boolean armRunningToPosition = false;
    private double armPower = 0;

    private double getArmAngleInRadians() {
        // Convert current encoder position to angle relative to vertical (90 degrees)
        double encoderDifference = arm.getCurrentPosition() - ENCODER_VALUE_AT_90_DEGREES;
        return -(encoderDifference / ENCODER_TICKS_PER_REV) * angleCorrection * 2 * Math.PI;
    }


    private double calculateArmPID(double targetPosition) {
        double currentTime = pidTimer.seconds();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        double currentPosition = arm.getCurrentPosition();
        double error = targetPosition - currentPosition;

        // Limit integral windup
        if (Math.abs(error) < 100) {
            integralSum += error * deltaTime;
        } else {
            integralSum = 0;
        }

        double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0;
        lastError = error;
        telemetry.addData("Derivative", derivative);

        // Gravity compensation
        double angleFromVertical = getArmAngleInRadians();
        double gravityComp = k2G * Math.sin(angleFromVertical);

        // PID calculation
        double output = (error * kP) + (integralSum * kI) + (derivative * kD) + gravityComp;
        output = Math.min(Math.max(output, -maxPIDPower), maxPIDPower);
        output += gravityComp;

        // Limit motor power for safety
        return Math.min(1, Math.max(output, -1));
    }

    @Override
    public void runOpMode() {
        liftL = hardwareMap.get(DcMotorEx.class, "lift_left");
        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        double liftLPos = liftL.getCurrentPosition();
        double liftRPos = liftR.getCurrentPosition();
        double armPos = arm.getCurrentPosition();
        double clawPos = claw.getPosition();


        telemetry.addData("Left Lift Position:", liftLPos);
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

            liftLPos = liftL.getCurrentPosition();
            liftRPos = liftR.getCurrentPosition();
            armPos = arm.getCurrentPosition();
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
            if (liftRunningToPosition && !liftR.isBusy() && !liftL.isBusy()) {
                // Motor has reached its target
                liftRunningToPosition = false;
                liftL.setPower(0); // Stop motor
                liftR.setPower(0);
                liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (!liftRunningToPosition) {
                double trigger = gamepad1.right_trigger - gamepad1.left_trigger; // number between -1 and 1

                maxPower = Math.max(0, Math.min(maxPower, 1));
                double powerApplied = trigger * maxPower;
                liftL.setPower(powerApplied);
                liftR.setPower(powerApplied);
                telemetry.addData("Triggers:", trigger);
                telemetry.addData("Current Power:", powerApplied);
                telemetry.addData("Max Power Multiplier", maxPower);
            }

            // Arm control
            if (gamepad1.y) {
                armTargetPosition = -1000; // Test position 1
                armRunningToPosition = true;
            } else if (gamepad1.a) {
                armTargetPosition = -200; // Test position 2
                armRunningToPosition = true;
            } else if (gamepad1.b) {
                armTargetPosition = ENCODER_VALUE_AT_90_DEGREES; // Vertical position
                armRunningToPosition = true;
            }

            // Manual override with joystick
            double leftStickY = -gamepad1.left_stick_y; // Joystick input for manual control

            if (Math.abs(leftStickY) > 0.1) {
                // Manual mode: Disable PID if more than 0.1 stick applied and apply manual power with gravity compensation
                armRunningToPosition = false;
                double angleFromVertical = getArmAngleInRadians();
                double gravityComp = kG * Math.sin(angleFromVertical);
                armPower = Math.min(Math.max(leftStickY * maxPower + gravityComp, -1), 1);
            } else if (armRunningToPosition) {
                // PID mode: Move to a target position
                armTargetPosition = Math.max(Math.min(armTargetPosition, MIN_ENCODER_VALUE), MAX_ENCODER_VALUE);
                armPower = calculateArmPID(armTargetPosition);
            } else {
                // Idle and Manual mode: Apply stick input and gravity compensation
                double angleFromVertical = getArmAngleInRadians();
                double gravityComp = kG * Math.sin(angleFromVertical);
                armPower = Math.min(Math.max(leftStickY * maxPower + gravityComp, -1), 1);
            }

            if (gamepad1.x) {
                arm.setPower(0);
            } else {
                arm.setPower(armPower);
            }
            // add arm power and gravity compensation to telemetry

            telemetry.addData("Arm Power:", armPower);
            telemetry.addData("Gravity Comp:", kG * Math.sin(getArmAngleInRadians()));
            telemetry.addData("Arm Angle (deg):", Math.toDegrees(getArmAngleInRadians()));

            double rightStickY = -gamepad1.right_stick_y;
            double newClawPos = claw.getPosition() + rightStickY / 50;
            newClawPos = Math.min(1.0, Math.max(0.0, newClawPos));
            claw.setPosition(newClawPos);

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            telemetry.addData("Left Lift Position:", liftL.getCurrentPosition());
            telemetry.addData("Right Lift Position:", liftR.getCurrentPosition());
            telemetry.addData("(Right) Lift Velocity:", liftR.getVelocity());
            telemetry.addData("Lifts Encoder Difference:", liftR.getCurrentPosition() - liftL.getCurrentPosition());
            telemetry.addData("Is Lift Running to Position:", liftRunningToPosition);
            telemetry.addData("Either Lift Motor Busy:", liftL.isBusy() || liftR.isBusy());
            telemetry.addData("", "");
            telemetry.addData("Arm Position:", arm.getCurrentPosition());
            telemetry.addData("90 Degree Reference (Encoder):", ENCODER_VALUE_AT_90_DEGREES);
            telemetry.addData("Arm Velocity (Encoder):", arm.getVelocity());
            telemetry.addData("Arm Angular Velocity (deg/s):", Math.toDegrees(arm.getVelocity() / ENCODER_TICKS_PER_REV * 2 * Math.PI));
            telemetry.addData("Arm Target Position:", armTargetPosition);
            telemetry.addData("Is Arm Running to Position:", armRunningToPosition);
            telemetry.addData("", "");
            telemetry.addData("Claw Position:", claw.getPosition());
            telemetry.addData("", "");
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
