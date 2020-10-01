package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HowlersMotor;

public class DriveTrain {

    private final MecanumDrive driveTrain;

    public DriveTrain(final HardwareMap hwMap, final MecanumDrive mecanumDrive, HowlersHardware robot) {
        robot.rightFront = new HowlersMotor(hwMap, "rightFront", 145.6);
        robot.rightBack = new HowlersMotor(hwMap, "rightBack", 145.6);
        robot.leftBack = new HowlersMotor(hwMap, "leftBack", 145.6);
        robot.leftFront = new HowlersMotor(hwMap, "leftFront", 145.6);
        robot.mecanumDrive = new MecanumDrive(robot.leftFront, robot.rightFront, robot.leftBack, robot.rightBack);

        driveTrain = mecanumDrive;
    }

    public void drive(double strafeSpeed, double forwardSpeed, double rotationSpeed) {

        //drives based off of the robots orientation
        driveTrain.driveRobotCentric(strafeSpeed, forwardSpeed, rotationSpeed);
    }

    public void stop() {
        driveTrain.stop();
    }
}
