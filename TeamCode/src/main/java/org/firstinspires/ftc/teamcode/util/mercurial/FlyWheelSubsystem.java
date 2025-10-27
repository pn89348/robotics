package org.firstinspires.ftc.teamcode.util.mercurial;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
import kotlin.annotation.MustBeDocumented;

public class FlyWheelSubsystem implements Subsystem {
    //ALWAYS CHANGE THIS TO THE RIGHT CLASS
    public static final FlyWheelSubsystem INSTANCE = new FlyWheelSubsystem();
    public static DcMotorEx m;
    public static boolean FlywheelIsMoving;
    static double power = 0.7;
    Telemetry telemetry;
    private FlyWheelSubsystem() {
    }


    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach{}

    private Dependency<?> dependency =

            Subsystem.DEFAULT_DEPENDENCY
//ALWAYS CHANGE THIS TO THE RIGHT CLASS
                    .and(new SingleAnnotation<>(FlyWheelSubsystem.Attach.class));


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
        m = hmap.get(DcMotorEx.class,"flywheel");

        // other stuff for init of motors/ servos go here


    }
    //not working rn
    public void PreUserLoopHook(@NonNull Wrapper opMode) {
        //telemetry = FeatureRegistrar.getActiveOpMode().telemetry;
    }


    public static double getPower(){
        return power;
    }
    //commands

    // not working rn
//    public  Lambda Telemetry() {
//        return new Lambda("telemetry")
//                .setInit(() -> {
//                })
//                .setExecute(() -> {
//                    telemetry.addData("Power" ,power);
//                    telemetry.update();
//                })
//                .setEnd(interrupted -> {
//
//                })
//                .setFinish(() -> {
//                    return false;
//                })
//                .setInterruptible(true)
//                .setRunStates(Wrapper.OpModeState.ACTIVE);
//    }
    public static boolean IsFlywheelOn() {
        return FlywheelIsMoving;
    }

    public static Lambda Shoot() {
        return new Lambda("Shoot Artifact")
                .setInit(() -> {
                    m.setPower(power);
                    FlywheelIsMoving = true;
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
    public static Lambda stop() {
        return new Lambda("stop flywheel")
                .setInit(() -> {

                    m.setPower(0);
                    FlywheelIsMoving = false;
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
    public static Lambda updatePower(double interval){
        return new Lambda("update flywheel power")
                .setInit(() -> {
                    power += interval;
                    if (power > 1) power = 1;
                    if (power < 0) power = 0;
                })
                .setExecute(() -> {

                })
                .setEnd(interrupted -> {

                })
                .setFinish(() -> {
                    return true;
                })
                .setInterruptible(true)
                .setRunStates(Wrapper.OpModeState.ACTIVE);
    }
}
