package org.firstinspires.ftc.teamcode.hardwaremaps;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HowlersMotor;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.WobbleArm.WobbleArm;

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

    public HowlersMotor intakeMotor = null;
    public HowlersMotor feederMotor = null;

    public HowlersMotor wobbleGoal = null;

    public Turret turret = null;
    public DriveTrain driveTrain = null;
    public Intake intake = null;
    public WobbleArm wobbleArm = null;

    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    public enum SubsystemType {
        DRIVE_TRAIN,
        CAMERA,
        INTAKE,
        TURRET,
        WOBBLE_ARM,
    }

    @Config
    public static class RobotConstants {
        public static double flywheelP = 10;
        public static double flywheelI = 0;
        public static double flywheelD = 0.01;
        public static double flywheelF = 1;
        public static double SPEED_OVERRIDE = 0;
        public static double flywheelSETPOINT = 0;
        public static double flywheelTOLERANCE = 0.01;
        public static boolean invertFlywheel = true;
        public static boolean displayPID = false;
    }

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

    public void init(HardwareMap ahwMap, boolean initDrivetrain, boolean initTurret, boolean initIntake, boolean initWobbleGoal) {
        hwMap = ahwMap;

        if(initDrivetrain) {
            driveTrain = new DriveTrain(hwMap);
            driveTrain.setInverted(true);
        }
        if(initTurret) turret = new Turret(hwMap);
        if(initIntake) intake = new Intake(hwMap);
        if(initWobbleGoal) wobbleArm = new WobbleArm(hwMap);

    }
    public Subsystem getSubsystem(SubsystemType subsystem) {
        Subsystem subsystemToReturn;
        switch (subsystem) {
            case DRIVE_TRAIN: subsystemToReturn = driveTrain;
            case INTAKE: subsystemToReturn = intake;
            case TURRET: subsystemToReturn = turret;
            case WOBBLE_ARM: subsystemToReturn = wobbleArm;
            default: subsystemToReturn = null;
        }
        return subsystemToReturn;
    }
}
