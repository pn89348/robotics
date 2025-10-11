package org.firstinspires.ftc.teamcode.util.mercurial;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

public class ServoSubsystem implements Subsystem {

    public static final ServoSubsystem INSTANCE = new ServoSubsystem();
    public static Servo s;
    private ServoSubsystem() {
    }


    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach{}

    private Dependency<?> dependency =

            Subsystem.DEFAULT_DEPENDENCY

                    .and(new SingleAnnotation<>(Attach.class));


    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }


//    private final SubsystemObjectCell<Servo> s = subsystemCell(() -> {
//
//        Servo s = FeatureRegistrar.getActiveOpMode().hardwareMap.get(Servo.class, "servo");
//
//        return s;
//    });
//
//    public static Servo getServo() {
//        return INSTANCE.s.get();
//    }

    public void preUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hmap = opMode.getOpMode().hardwareMap;
        s = hmap.get(Servo.class,"servo");
        // default command should be set up here, not in the constructor
//        setDefaultCommand(MoveServo1(0));

    }



    public static Lambda MoveServo1(double pos) {
        return new Lambda("Move Servo Pos:" + pos)
                .setInit(() -> {

                    s.setPosition(pos);
                })
                .setExecute(() -> {

                })
                .setEnd(interrupted -> {

                })
                .setFinish(() -> {
                    return true;
                })
                .setInterruptible(false)
                .setRequirements(INSTANCE)
                .setRunStates(Wrapper.OpModeState.ACTIVE);
    }
}



