package org.firstinspires.ftc.teamcode.util.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mercurial.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.util.mercurial.KickerSubsystem;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.groups.Advancing;

@Mercurial.Attach
@KickerSubsystem.Attach
@FlyWheelSubsystem.Attach
@TeleOp(name = "Kicker + flywheel")
public class KickerAndFlyWheel extends OpMode {
    @Override
    public void init() {
        BoundGamepad gamepad2 = Mercurial.gamepad2();

        gamepad2.leftBumper().onTrue(new Advancing(FlyWheelSubsystem.Shoot(),FlyWheelSubsystem.stop()));
        gamepad2.rightBumper().onTrue(new Advancing(KickerSubsystem.kick(),KickerSubsystem.defaultpos()));
    }

    @Override
    public void loop() {

    }
}
