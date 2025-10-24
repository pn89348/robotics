package org.firstinspires.ftc.teamcode.util.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mercurial.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.util.mercurial.KickerSubsystem;
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
public class TeleOpTest extends OpMode {


    @Override
    public void init() {
        BoundGamepad gamepad2 = Mercurial.gamepad2();
        // Needed Controls
        gamepad2.leftBumper().onTrue(new Advancing(intakeRollerSubsystem.SpinIntake(),intakeRollerSubsystem.StopIntake()));
        gamepad2.a().onTrue(new Advancing(intakeRollerSubsystem.Extake(),intakeRollerSubsystem.StopIntake()));

        Sequential shootSequence = new Sequential(
                FlyWheelSubsystem.Shoot(),
                new Wait(0.5),
                KickerSubsystem.kickStart(),
                new Wait(1.5)
        );
        Sequential stopSequence = new Sequential(
                KickerSubsystem.kickstop(),
                new Wait(0.5),
                FlyWheelSubsystem.stop()
        );
        gamepad2.rightBumper().onTrue(new Sequential(shootSequence,stopSequence));
        // Overrides
            // Kicker
        gamepad2.b().onTrue(new Sequential(KickerSubsystem.kickStart(),new Wait(1),KickerSubsystem.kickstop()));
            // Flywheel
        gamepad2.x().onTrue(new Sequential(FlyWheelSubsystem.Shoot(),new Wait(0.5),FlyWheelSubsystem.stop()));




    }

    @Override
    public void loop() {

    }
}
