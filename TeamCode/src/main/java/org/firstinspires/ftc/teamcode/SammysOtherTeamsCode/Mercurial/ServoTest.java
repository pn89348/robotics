package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.Mercurial;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.groups.Advancing;
@Mercurial.Attach
@ServoSubsystem.Attach
@TeleOp(name = "servoTest")
public class ServoTest extends OpMode {
    @Override
    public void init() {
        BoundGamepad Gamepad2 = Mercurial.gamepad2();

        Gamepad2.rightBumper().onTrue (ServoSubsystem.MoveServo1(0));
        Gamepad2.leftBumper().onTrue(ServoSubsystem.MoveServo1(1));
    }

    @Override
    public void loop() {

    }
}
