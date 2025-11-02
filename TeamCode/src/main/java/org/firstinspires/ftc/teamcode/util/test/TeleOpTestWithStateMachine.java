  package org.firstinspires.ftc.teamcode.util.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mercurial.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.util.mercurial.KickerSubsystem;
import org.firstinspires.ftc.teamcode.util.mercurial.Robot;
import org.firstinspires.ftc.teamcode.util.mercurial.drivetrainSubsystem;
import org.firstinspires.ftc.teamcode.util.mercurial.intakeRollerSubsystem;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.groups.Advancing;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Mercurial.Attach
@drivetrainSubsystem.Attach
@intakeRollerSubsystem.Attach
@KickerSubsystem.Attach
@FlyWheelSubsystem.Attach
@TeleOp(name = "TeleOpTest")
public class TeleOpTestWithStateMachine extends OpMode {


    @Override
    public void init() {



        BoundGamepad gamepad2 = Mercurial.gamepad2();
        BoundGamepad gamepad1 = Mercurial.gamepad1();

        //Gamepad 2
        // Needed Controls
        gamepad2.leftBumper().onTrue(new Advancing(intakeRollerSubsystem.SpinIntake(),intakeRollerSubsystem.StopIntake()));
        gamepad2.a().onTrue(new Advancing(intakeRollerSubsystem.Extake(),intakeRollerSubsystem.StopIntake()));


        gamepad2.rightBumper().onTrue(new Sequential(FlyWheelSubsystem.Shoot(),
                new Wait(2), KickerSubsystem.kick(), new Wait(0.5),
                KickerSubsystem.defaultpos()));
        // Overrides
            // Kicker
        gamepad2.b().onTrue(new Sequential(KickerSubsystem.kick(),new Wait(0.5),KickerSubsystem.defaultpos()));
            // Flywheel
        gamepad2.x().onTrue(new Advancing(FlyWheelSubsystem.Shoot(),FlyWheelSubsystem.stop()));

        gamepad2.dpadUp().onTrue(FlyWheelSubsystem.updatePower(0.05));



        gamepad2.dpadDown().onTrue(FlyWheelSubsystem.updatePower(-0.05));
        //Gamepad 1
        gamepad1.dpadUp().onTrue(FlyWheelSubsystem.updatePower(0.05));
        gamepad1.dpadDown().onTrue(FlyWheelSubsystem.updatePower(-0.05));
        gamepad1.leftBumper().onTrue(new Advancing(intakeRollerSubsystem.SpinIntake(),intakeRollerSubsystem.StopIntake()));
    }

    @Override
    public void loop() {

        telemetry.addData("Flywheel power:", FlyWheelSubsystem.getPower());
        telemetry.addData("flywheel status:", FlyWheelSubsystem.IsFlywheelOn());

    }
}
