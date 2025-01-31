package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "V4 Arm PID, Lifts, Claw", group = "Testing and Tuning")
public class ArmLiftClawTest4 extends LinearOpMode {
        private ElapsedTime runtime = new ElapsedTime();
        public static boolean log = true;
        private double oldTime = 0;
        private boolean prevRightBumper = false;
        private boolean prevLeftBumper = false;
        private boolean prevX = false;
        private boolean prevB = false;

        private DcMotorEx liftL = null;
        private DcMotorEx liftR = null;
        private DcMotorEx arm = null;
        private Servo claw = null;
        private boolean liftRunningToPosition = false;
        double liftMaxPower = 1;
        double armMaxPower = 0.75;
        double powerChangeSensitivity = 0.25;
        public static int maxLiftHeight = 3100;
        public static int minLiftHeight = 0;
        public static int liftDown = 10;
        private int liftTargetPosition = 0;
        public static int liftAscent = maxLiftHeight;
        public static int liftBasket = maxLiftHeight;
        public static double clawOpen = 0.2833; // slightly wider than horizontal sample
        public static double clawClose = 0.5; // enough to hold sample in smallest orientation
        // arm pid variables
        private static final double ARM_ENCODER_TICKS_PER_REV = 1992.6;
        public static int armZeroOffset = 0;
        private static int armVertical = -647;
        private static int armAbsMin = 0; // Starting position
        private static int armSafeMin = -100;
        private static int armSafeMax = -1305;
        public static int armSample = -1305;
        public static int armSub = -1100;
        public static int armBasket = 700;
        private double armEncoderAngleCorrection = 90.0 / 80.0;
        // Refined PID Constants
        public static double kPArm = 0.0025;  // Conservative starting value
        public static double kIArm = 0.0009;    // Start with no integral term
        public static double kDArm = 0.0005; // Conservative derivative
        public static double kGArm = 0.12;   // Gravity compensation
        public static double k2GArm = 0.19; // Gravity comp for PID function, kinetic kG
        private double integralSum = 0;
        private double lastError = 0;
        public static double maxArmPIDPower = 0.7;
        private ElapsedTime pidTimer = new ElapsedTime();
        private double lastTime = 0;
        private double armTargetPosition = -200;
        private boolean armRunningToPosition = false;
        private double armPower = 0;

        private double getArmAngleInRadians() {
            // Convert current encoder position to angle relative to vertical (90 degrees)
            double encoderDifference = arm.getCurrentPosition() + armZeroOffset - armVertical;
            return -(encoderDifference / ARM_ENCODER_TICKS_PER_REV) * armEncoderAngleCorrection * 2 * Math.PI;
        }


        private double calculateArmPID(double targetPosition, double maxPIDPower, double kP, double kI, double kD, double kG) {
            double currentTime = pidTimer.seconds();
            double deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            double currentPosition = arm.getCurrentPosition() + armZeroOffset;
            double error = targetPosition - currentPosition;

            // Limit integral windup
            if (Math.abs(error) < 150) {
                integralSum += error * deltaTime;
            } else {
                integralSum = 0;
            }

            double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0;
            lastError = error;
            telemetry.addData("Derivative", derivative);

            // Gravity compensation
            double angleFromVertical = getArmAngleInRadians();
            double gravityComp = kG * Math.sin(angleFromVertical);

            // PID calculation
            double output = (error * kP) + (integralSum * kI) + (derivative * kD) + gravityComp;
            output = Math.min(Math.max(output, -maxPIDPower), maxPIDPower);
            output += gravityComp;

            // Limit motor power for safety
            return Math.min(1, Math.max(output, -1));
        }
        private double calculateArmPID(double targetPosition, double maxPIDPower, double kP, double kI, double kD) {
            return calculateArmPID(targetPosition, maxPIDPower, kP, kI, kD, k2GArm);
        }

        private double calculateArmPID(double targetPosition, double maxPIDPower) {
            return calculateArmPID(targetPosition, maxPIDPower, kPArm, kIArm, kDArm, k2GArm);
        }

        private double calculateArmPID(double targetPosition) {
            return calculateArmPID(targetPosition, maxArmPIDPower, kPArm, kIArm, kDArm, k2GArm);
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
                boolean rightBumper = gamepad2.right_bumper;
                boolean leftBumper = gamepad2.left_bumper;
                boolean x = gamepad2.x;
                boolean b = gamepad2.b;
//                if (prevRightBumper && !rightBumper) { // on release of button
//                    liftMaxPower += powerChangeSensitivity;
//                    armMaxPower += powerChangeSensitivity;
//                }
//                if (prevLeftBumper && !leftBumper) {
//                    liftMaxPower -= powerChangeSensitivity;
//                    armMaxPower -= powerChangeSensitivity;
//                }
//                liftMaxPower = Math.min(Math.max(liftMaxPower, 0), 1);
//                armMaxPower = Math.min(Math.max(armMaxPower, 0), 1);
                if (rightBumper && !prevRightBumper) { // on release of button
                    claw.setPosition(clawClose);
                }
                if (leftBumper && !prevLeftBumper) {
                    claw.setPosition(clawOpen);
                }

//                if (x && !prevX) {
//                    claw.setPosition(clawClose);
//                }
//                if (b && !prevB) {
//                    claw.setPosition(clawOpen);
//                }
                prevRightBumper = rightBumper;
                prevLeftBumper = leftBumper;
                prevX = x;
                prevB = b;

                liftLPos = liftL.getCurrentPosition();
                liftRPos = liftR.getCurrentPosition();
                armPos = arm.getCurrentPosition();
                clawPos = claw.getPosition();

                if (gamepad2.dpad_left) {
                    liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftRunningToPosition = false;
                } else if (gamepad2.dpad_up) {
                    liftTargetPosition = liftBasket;
                    liftL.setTargetPosition(liftTargetPosition);
                    liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftL.setPower(liftMaxPower);
                    liftR.setTargetPosition(liftTargetPosition);
                    liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftR.setPower(liftMaxPower);
                    liftRunningToPosition = true;
                } else if (gamepad2.dpad_down) {
                    liftTargetPosition = liftDown;
                    liftL.setTargetPosition(liftTargetPosition);
                    liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftL.setPower(liftMaxPower);
                    liftR.setTargetPosition(liftTargetPosition);
                    liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftR.setPower(liftMaxPower);
                    liftRunningToPosition = true;
                } else if (gamepad2.dpad_right) {
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
                    if (liftTargetPosition == liftBasket && gamepad2.left_trigger < 0.1 && gamepad2.right_trigger < 0.1) {
                        liftL.setTargetPosition(liftTargetPosition);
                        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftL.setPower(liftMaxPower);
                        liftR.setTargetPosition(liftTargetPosition);
                        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftR.setPower(liftMaxPower);
                    }
                    else {
                        liftRunningToPosition = false;
                        liftL.setPower(0); // Stop motor
                        liftR.setPower(0);
                        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                }

                if (!liftRunningToPosition) {
                    double trigger = gamepad2.right_trigger - gamepad2.left_trigger; // number between -1 and 1

                    liftMaxPower = Math.max(0, Math.min(liftMaxPower, 1));
                    double powerApplied = trigger * liftMaxPower;
                    liftL.setPower(powerApplied);
                    liftR.setPower(powerApplied);
                }

                // Arm control
                if (gamepad2.y) {
                    armTargetPosition = armVertical; // Test position 1
                    armRunningToPosition = true;
                } else if (gamepad2.x) {
                    armTargetPosition = armSub;
                    armRunningToPosition = true;
                } else if (gamepad2.a) {
                    armTargetPosition = armSample; // Test position 2
                    armRunningToPosition = true;
                }

                // Manual override with joystick
                double leftStickY = -gamepad2.left_stick_y; // Joystick input for manual control

                if (Math.abs(leftStickY) > 0.1) {
                    // Manual mode: Disable PID if more than 0.1 stick applied and apply manual power with gravity compensation
                    armRunningToPosition = false;
                    double angleFromVertical = getArmAngleInRadians();
                    double gravityComp = kGArm * Math.sin(angleFromVertical);
                    armPower = Math.min(Math.max(leftStickY * armMaxPower + gravityComp, -1), 1);
                } else if (armRunningToPosition) {
                    // PID mode: Move to a target position
                    armTargetPosition = Math.max(Math.min(armTargetPosition, armSafeMin), armSafeMax);
                    if (armTargetPosition == armSample) {
                        armPower = calculateArmPID(armTargetPosition, maxArmPIDPower, kPArm, kIArm, kDArm, kGArm);
                    } else {
                        armPower = calculateArmPID(armTargetPosition);
                    }
                } else {
                    // Idle and Manual mode: Apply stick input and gravity compensation
                    double angleFromVertical = getArmAngleInRadians();
                    double gravityComp = kGArm * Math.sin(angleFromVertical);
                    armPower = Math.min(Math.max(leftStickY * armMaxPower + gravityComp, -1), 1);
                }

                if (gamepad2.b) {
                    arm.setPower(0);
                } else {
                    arm.setPower(armPower);
                }

                double rightStickY = -gamepad2.right_stick_y;
                double newClawPos = claw.getPosition() + rightStickY / 50;
                newClawPos = Math.min(1.0, Math.max(0.0, newClawPos));
                claw.setPosition(newClawPos);

                double newTime = getRuntime();
                double loopTime = newTime - oldTime;
                double frequency = 1 / loopTime;
                if (log) {
                    telemetry.addData("Left Lift Position:", liftL.getCurrentPosition());
                    telemetry.addData("Right Lift Position:", liftR.getCurrentPosition());
                    telemetry.addData("(Right) Lift Velocity:", liftR.getVelocity());
                    telemetry.addData("Lifts Encoder Difference:", liftR.getCurrentPosition() - liftL.getCurrentPosition());
                    telemetry.addData("Is Lift Running to Position:", liftRunningToPosition);
                    telemetry.addData("Either Lift Motor Busy:", liftL.isBusy() || liftR.isBusy());
                    telemetry.addData("", "");
                    telemetry.addData("Arm Position (Raw):", arm.getCurrentPosition());
                    telemetry.addData("Arm Position (Adjusted):", arm.getCurrentPosition() + armZeroOffset);
                    telemetry.addData("Arm Angle (deg):", Math.toDegrees(getArmAngleInRadians()));
                    telemetry.addData("Arm Power:", armPower);
                    telemetry.addData("Gravity Comp:", kGArm * Math.sin(getArmAngleInRadians()));
//                    telemetry.addData("90 Degree Reference (Encoder):", armVertical);
//                    telemetry.addData("Arm Velocity (Encoder):", arm.getVelocity());
//                    telemetry.addData("Arm Angular Velocity (deg/s):", Math.toDegrees(arm.getVelocity() / ARM_ENCODER_TICKS_PER_REV * 2 * Math.PI));
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
    }
