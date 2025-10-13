package org.firstinspires.ftc.teamcode.util.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mercurial.ServoSubsystem;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;

@Mercurial.Attach
@ServoSubsystem.Attach
@TeleOp(name = "servoTest")
public class ServoTest extends OpMode {
    @Override
    public void init() {
        BoundGamepad Gamepad2 = Mercurial.gamepad2();

        Gamepad2.rightBumper().onTrue(ServoSubsystem.MoveServo1(0));
        Gamepad2.leftBumper().onTrue(ServoSubsystem.MoveServo1(1));

    }
    public void loop(){

    }
}