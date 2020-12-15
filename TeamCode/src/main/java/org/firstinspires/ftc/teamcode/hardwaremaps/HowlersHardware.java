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

    public HowlersMotor rightFront = null;
    public HowlersMotor rightBack = null;
    public HowlersMotor leftFront = null;
    public HowlersMotor leftBack = null;
    public MecanumDrive mecanumDrive = null;

    public HowlersMotor flywheel = null;

    public Turret turret = null;
    public DriveTrain driveTrain = null;


    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    public HowlersHardware(){

    }

    public void init(HardwareMap ahwMap, boolean initDrivetrain, boolean initTurret, boolean initIntake, boolean randomTesting) {
        hwMap = ahwMap;

        if(initDrivetrain) { driveTrain = new DriveTrain(hwMap, this); rightBack.setInverted(true); leftBack.setInverted(true); }
        if(initTurret) turret = new Turret(hwMap, this);
        if(randomTesting) {
             robot.rightFront = new HowlersMotor(hwMap, "one", 134.4);
                robot.rightBack = new HowlersMotor(hwMap, "two", 134.4);
        }
    }

}
