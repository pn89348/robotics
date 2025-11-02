package org.firstinspires.ftc.teamcode.util.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "flywheel velocity getter")
public class FlyWheelvelocity extends OpMode {
    DcMotorEx flywheel;
    @Override
    public void init() {
    flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");
    }

    @Override
    public void loop() {
    double power = gamepad1.left_trigger;
    flywheel.setVelocity((power*6000)*(28/60));
    double vel = flywheel.getVelocity();
    telemetry.addData("Velocity:",vel);
    }
}
