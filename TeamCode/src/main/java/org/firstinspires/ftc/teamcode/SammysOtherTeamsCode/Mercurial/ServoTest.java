package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.Mercurial;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.groups.Advancing;

public class ServoTest extends OpMode {
    @Override
    public void init() {
        BoundGamepad Gamepad2 = new BoundGamepad(new SDKGamepad(gamepad2));

        Gamepad2.rightBumper().onTrue
                (new Advancing(ServoSubsystem.MoveServo1(),ServoSubsystem.MoveServo2()));
    }

    @Override
    public void loop() {

    }
}
