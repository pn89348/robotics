package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.Mercurial;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;
import kotlin.annotation.MustBeDocumented;

public class ServoSubsystem implements Subsystem {
    public static final ServoSubsystem INSTANCE = new ServoSubsystem();

    private ServoSubsystem() {
    }

    // the annotation class we use to attach this subsystem
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach {
    }

    private Dependency<?> dependency =

            Subsystem.DEFAULT_DEPENDENCY

                    .and(new SingleAnnotation<>(LS.Attach.class));


    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    private final SubsystemObjectCell<Servo> s = subsystemCell(() -> {

        Servo s = FeatureRegistrar.getActiveOpMode().hardwareMap.get(Servo.class, "");

        return s;
    });

    public static Servo getServo() {
        return INSTANCE.s.get();
    }

    public void preUserInitHook(@NonNull Wrapper opMode) {
        // default command should be set up here, not in the constructor
        setDefaultCommand(MoveServo1());

    }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {

    }

    @Override
    public void preUserInitLoopHook(@NonNull Wrapper opMode) {
    }

    @Override
    public void preUserLoopHook(@NonNull Wrapper opMode) {
    }

    @Override
    public void postUserInitLoopHook(@NonNull Wrapper opMode) {
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {
    }


    @Override
    public void preUserStopHook(@NonNull Wrapper opMode) {
    }

    @Override
    public void postUserStopHook(@NonNull Wrapper opMode) {
    }


    @Override
    public void cleanup(@NonNull Wrapper opMode) {
    }
        Servo servo = getServo();
    public static Lambda MoveServo1() {
        return new Lambda("Move Servo Pos: 0.15")
                .setInit(() -> {
                    INSTANCE.servo.setPosition(0.15);
                })
                .setExecute(() -> {

                })
                .setEnd(interupted -> {

                })
                .setFinish(() -> {
                    return true;
                })
                .setInterruptible(false)
                .setRequirements(INSTANCE)
                .setRunStates(Wrapper.OpModeState.ACTIVE);
    }
    public static Lambda MoveServo2() {
        return new Lambda("Move Servo Pos: 0.3")
                .setInit(() -> {
                    INSTANCE.servo.setPosition(0.3);
                })
                .setExecute(() -> {

                })
                .setEnd(interupted -> {

                })
                .setFinish(() -> {
                    return true;
                })
                .setInterruptible(false)
                .setRequirements(INSTANCE)
                .setRunStates(Wrapper.OpModeState.ACTIVE);
    }
}


