package org.firstinspires.ftc.teamcode.util.mercurial;

import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.StateMachine;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.util.cell.RefCell;

public class Robot {

    public enum States {
        OUTTAKE_START,
        OUTTAKE_AUTOMATION,
        OUTTAKE_STOP,
        KICKER_UP,
        KICKER_DOWN,
        INTAKE_START,
        EXTAKE,
        INTAKE_STOP,
        DEFAULT
    }


    public static StateMachine<States> stateMachine = new StateMachine<>(States.DEFAULT)

            .withState(States.DEFAULT, (RefCell<States> state, String name) -> Lambda.from(
                    new Parallel(
                          FlyWheelSubsystem.stop(),
                          KickerSubsystem.defaultpos(),
                          intakeRollerSubsystem.StopIntake()

                    )
            ))
            .withState(States.EXTAKE,(state, name) -> Lambda.from(
                        intakeRollerSubsystem.Extake()
            ))
            .withState(States.INTAKE_START,(state, name) -> Lambda.from(
                    intakeRollerSubsystem.SpinIntake()
            ))
            .withState(States.INTAKE_STOP,(state, name) -> Lambda.from(
                    intakeRollerSubsystem.StopIntake()
            ))
            .withState(States.OUTTAKE_START,(state, name) -> Lambda.from(
                    FlyWheelSubsystem.Shoot()
            ))
            .withState(States.OUTTAKE_AUTOMATION,(state, name) -> Lambda.from(
                    new Sequential(FlyWheelSubsystem.Shoot(),
                            new Wait(2),
                            KickerSubsystem.kick(),
                            new Wait(0.5),
                            KickerSubsystem.defaultpos()
                    )
            ))
            .withState(States.OUTTAKE_STOP,(state, name) -> Lambda.from(
                    FlyWheelSubsystem.stop()
            ))
            .withState(States.KICKER_DOWN,(state, name) -> Lambda.from(
                    KickerSubsystem.defaultpos()
            ))
            .withState(States.KICKER_UP,(state, name) -> Lambda.from(
                    KickerSubsystem.kick()
            ));

    public static Lambda setState(States state) {
        return new Lambda("set state")
                .setInit(() -> {
                    if (stateMachine.getState().equals(States.OUTTAKE_AUTOMATION) && state.equals(States.OUTTAKE_START)) {
                        stateMachine.schedule(States.OUTTAKE_STOP);

                    }else{
                        stateMachine.schedule(state);
                    }
                })
                .setFinish(() -> {
                    return true;
                })
                .setInterruptible(true)
                .setRunStates(Wrapper.OpModeState.ACTIVE);
    }

}



