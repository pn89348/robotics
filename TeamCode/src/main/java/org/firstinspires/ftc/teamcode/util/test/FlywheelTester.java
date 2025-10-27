package org.firstinspires.ftc.teamcode.util.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp(name = "FlywheelTester")
public class FlywheelTester extends OpMode {
    DcMotorEx flywheel;
    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "intake");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
//        double power = gamepad2.left_trigger;
//        double velocity = (6000 *power)* (28/60); // max rpm of the motor is 6000
//        flywheel.setVelocity(velocity);
//        // velocity is the max rpm * power to find the percentage, and then
//        // multiply by ticks per second to find overall ticks per sec
//        telemetry.addData("Velocity", flywheel.getVelocity());
//        telemetry.addData("Power", power);

        double power = gamepad2.right_trigger;
        flywheel.setPower(power);
        telemetry.addData("Power", flywheel.getPower());
        telemetry.addData("Velocity", flywheel.getVelocity());

    }
}
