package org.firstinspires.ftc.teamcode.util.mercurial;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.groups.Advancing;


@Mercurial.Attach
@intakeRollerSubsystem.Attach
@TeleOp(name = "intakeSubsystemTest")
public class intakeTest extends OpMode {
    BoundGamepad gamepad2 = Mercurial.gamepad2();

    @Override
    public void init() {
        gamepad2.leftBumper().onTrue(new Advancing(intakeRollerSubsystem.SpinIntake(),intakeRollerSubsystem.StopIntake()));
    }



    @Override
    public void loop() {

    }
}