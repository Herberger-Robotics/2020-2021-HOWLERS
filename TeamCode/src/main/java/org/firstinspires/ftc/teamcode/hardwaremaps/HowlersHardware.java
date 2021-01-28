package org.firstinspires.ftc.teamcode.hardwaremaps;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HowlersMotor;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Turret.Turret;

import java.lang.reflect.Array;
import java.util.List;

public class HowlersHardware {
    // static variable single_instance of type Singleton
    private static HowlersHardware instance = null;

    public HowlersMotor rightFront = null;
    public HowlersMotor rightBack = null;
    public HowlersMotor leftFront = null;
    public HowlersMotor leftBack = null;
    public MecanumDrive mecanumDrive = null;

    public HowlersMotor flywheel = null;

    public HowlersMotor intake = null;

    public Turret turret = null;
    public DriveTrain driveTrain = null;

    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    // private constructor restricted to this class itself
    private HowlersHardware() {

    }

    // static method to create instance of Singleton class
    public static HowlersHardware getInstance() {
        if (instance == null)
            instance = new HowlersHardware();

        return instance;
    }

    public static void yeetTheInstance() {
        instance = null;
    }

    public static HowlersHardware resetInstance() {
        instance = new HowlersHardware();
        return instance;
    }

    public void init(HardwareMap ahwMap, boolean initDrivetrain, boolean initTurret, boolean initIntake) {
        hwMap = ahwMap;

        if(initDrivetrain) { driveTrain = new DriveTrain(hwMap); rightBack.setInverted(true); leftBack.setInverted(true); }
        if(initTurret) turret = new Turret(hwMap);
        if(initIntake) intake = new HowlersMotor(hwMap, "intakeMotor", 134.4);
    }
}
