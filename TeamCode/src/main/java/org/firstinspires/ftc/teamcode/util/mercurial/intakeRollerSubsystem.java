package org.firstinspires.ftc.teamcode.util.mercurial;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class intakeRollerSubsystem implements Subsystem {
    //ALWAYS CHANGE THIS TO THE RIGHT CLASS
    public static final intakeRollerSubsystem INSTANCE = new intakeRollerSubsystem();
    public static DcMotorEx Roller;
    private intakeRollerSubsystem() {
    }


    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach{}

    private Dependency<?> dependency =

            Subsystem.DEFAULT_DEPENDENCY
//ALWAYS CHANGE THIS TO THE RIGHT CLASS
                    .and(new SingleAnnotation<>(intakeRollerSubsystem.Attach.class));


    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    //init hook
    public void preUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hmap = opMode.getOpMode().hardwareMap;

        Roller = hmap.get(DcMotorEx.class,"intake");
        // other stuff for init of motors/ servos go here

    }

    //commands

    public static Lambda SpinIntake() {
        return new Lambda("spin intake")
                .setInit(() -> {
                    Roller.setPower(1);
                })
                .setExecute(() -> {

                })
                .setEnd(interrupted -> {

                })
                .setFinish(() -> {
                    return true;
                })
                .setInterruptible(true)
                .setRequirements(INSTANCE)
                .setRunStates(Wrapper.OpModeState.ACTIVE);
    }
    public static Lambda StopIntake() {
        return new Lambda("stop intake")
                .setInit(() -> {
                    Roller.setPower(0);
                })
                .setExecute(() -> {

                })
                .setEnd(interrupted -> {

                })
                .setFinish(() -> {
                    return true;
                })
                .setInterruptible(true)
                .setRequirements(INSTANCE)
                .setRunStates(Wrapper.OpModeState.ACTIVE);
    }

}


