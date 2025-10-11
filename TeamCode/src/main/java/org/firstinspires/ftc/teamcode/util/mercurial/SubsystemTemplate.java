package org.firstinspires.ftc.teamcode.util.mercurial;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

public class SubsystemTemplate implements Subsystem {
    //ALWAYS CHANGE THIS TO THE RIGHT CLASS
    public static final SubsystemTemplate INSTANCE = new SubsystemTemplate();
    public static Servo s;
    public static DcMotorEx m;
    private SubsystemTemplate() {
    }


    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach{}

    private Dependency<?> dependency =

            Subsystem.DEFAULT_DEPENDENCY
//ALWAYS CHANGE THIS TO THE RIGHT CLASS
                    .and(new SingleAnnotation<>(SubsystemTemplate.Attach.class));


    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

//One WAY TO INIT SERVOS/ MOTORS(change a little bit of the code for motor)


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
        m = hmap.get(DcMotorEx.class,"motor");
        // other stuff for init of motors/ servos go here



    }



    public static Lambda MoveServo(double pos) {
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
    public static Lambda MoveMotor(int pos) {
        return new Lambda("Move Motor Pos:" + pos)
                .setInit(() -> {

                    m.setTargetPosition(pos);
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
