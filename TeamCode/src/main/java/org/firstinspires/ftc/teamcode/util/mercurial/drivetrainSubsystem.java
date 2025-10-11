package org.firstinspires.ftc.teamcode.util.mercurial;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
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

public class drivetrainSubsystem implements Subsystem {

    public static final drivetrainSubsystem INSTANCE = new drivetrainSubsystem();
    public static DcMotorEx fr,fl,br,bl;
    static double drive,turn,strafe,FLspeed,FRspeed,BLspeed,BRspeed;

    private drivetrainSubsystem() {
    }


    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach{}

    private Dependency<?> dependency =

            Subsystem.DEFAULT_DEPENDENCY

                    .and(new SingleAnnotation<>(drivetrainSubsystem.Attach.class));


    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }



    public void preUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hmap = opMode.getOpMode().hardwareMap;
        fr = hmap.get(DcMotorEx.class,"right_front");
        fl = hmap.get(DcMotorEx.class,"left_front");
        bl = hmap.get(DcMotorEx.class,"left_back");
        br = hmap.get(DcMotorEx.class,"right_back");

        // other stuff for init of motors/ servos go here
        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //3 reversed cus we accidentally flipped one of the wires prob need to fix soon
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.FORWARD);
        br.setDirection(DcMotorEx.Direction.REVERSE);



    }




    public static Lambda MoveMotor(Gamepad gp1) {
        return new Lambda("DriveTrain")
                .setInit(() -> {


                })
                .setExecute(() -> {
                    //Drive Train code Start===============================================================
                    drive = (gp1.left_stick_y * -1);
                    turn = (gp1.right_stick_x);
                    strafe = (gp1.left_stick_x) ;

                    FLspeed = drive + turn + strafe;
                    FRspeed = drive - turn - strafe;
                    BLspeed = drive + turn - strafe;
                    BRspeed = drive - turn + strafe;

                    // Scaling Drive Powers Proportionally
                    double maxF = Math.max((abs(FLspeed)),(abs(FRspeed)));
                    double maxB = Math.max((abs(BLspeed)),(abs(BRspeed)));
                    double maxFB_speed = Math.max(abs(maxF), abs(maxB));

                    if(maxFB_speed > 1){
                        FLspeed = FLspeed / maxFB_speed;
                        FRspeed = FRspeed / maxFB_speed;
                        BLspeed = BLspeed / maxFB_speed;
                        BRspeed = BRspeed / maxFB_speed;
                    }

                    fl.setPower(FLspeed);
                   fr.setPower(FRspeed);
                    bl.setPower(BLspeed);
                    br.setPower(BRspeed);


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

