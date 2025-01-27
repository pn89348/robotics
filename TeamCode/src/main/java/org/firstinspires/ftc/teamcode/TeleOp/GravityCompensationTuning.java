package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name = "kG Tuning", group = "Testing and Tuning")
public class GravityCompensationTuning extends LinearOpMode {
    private DcMotorEx arm;
    private static final double ENCODER_TICKS_PER_REV = 1993.6;
    private static final int VERTICAL_ENCODER = 0; // Set this to value from calibration
    private double kG = 0.15;

    private double getArmAngleInRadians() {
        double encoderDifference = arm.getCurrentPosition() - VERTICAL_ENCODER;
        return (encoderDifference / ENCODER_TICKS_PER_REV) * 2 * Math.PI;
    }

    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Instructions", "Move arm to horizontal");
        telemetry.addData("Use bumpers", "to adjust kG");
        telemetry.addData("Goal", "Arm should hold position without power");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Manual control
            double leftStickY = -gamepad1.left_stick_y;
            if (Math.abs(leftStickY) > 0.1) {
                arm.setPower(leftStickY * 0.3);
            } else {
                // Apply gravity compensation only
                double angleFromVertical = getArmAngleInRadians();
                double gravityComp = kG * Math.sin(angleFromVertical);
                arm.setPower(gravityComp);
            }

            // Adjust kG
            if (gamepad1.right_bumper) {
                kG += 0.005;
            }
            if (gamepad1.left_bumper) {
                kG -= 0.005;
            }

            telemetry.addData("Current kG", "%.3f", kG);
            telemetry.addData("Angle (deg)", "%.1f", Math.toDegrees(getArmAngleInRadians()));
            telemetry.addData("Encoder", arm.getCurrentPosition());
            telemetry.update();
        }
    }
}
