package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HowlersMotor;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

public class DriveTrain extends Subsystem {

    private final MecanumDrive driveTrain;

    private double speed = 1;

    public double setSpeed(double setter) {
        speed = setter;
        return speed;
    }

    public double getSpeed() {
        return speed;
    }

    public DriveTrain(final HardwareMap hwMap) {
        HowlersHardware robot = HowlersHardware.getInstance();
        robot.rightFront = new HowlersMotor(hwMap, "rightFront", 134.4);
        robot.rightBack = new HowlersMotor(hwMap, "rightBack", 134.4);
        robot.leftBack = new HowlersMotor(hwMap, "leftBack", 134.4);
        robot.leftFront = new HowlersMotor(hwMap, "leftFront", 134.4);
        robot.mecanumDrive = new MecanumDrive(robot.leftFront, robot.rightFront, robot.leftBack, robot.rightBack);

        driveTrain = robot.mecanumDrive;
    }

    public void drive(double strafeSpeed, double forwardSpeed, double rotationSpeed) {

        //drives based off of the robots orientation
        driveTrain.driveRobotCentric(strafeSpeed, forwardSpeed, rotationSpeed);
    }

    public void setInverted(boolean inverted) {
        HowlersHardware robot = HowlersHardware.getInstance();
        robot.rightBack.setInverted(inverted);
        robot.rightFront.setInverted(inverted);
        robot.leftBack.setInverted(inverted);
        robot.leftFront.setInverted(inverted);
    }

    public boolean isBusy() {
        HowlersHardware robot = HowlersHardware.getInstance();
        boolean isBusy;
        if(robot.rightFront.busy() && robot.leftBack.busy() && robot.rightBack.busy() && robot.leftFront.busy()) isBusy = true;
        else isBusy = false;
        return isBusy;
    }

    public void set(double speed) {
        HowlersHardware robot = HowlersHardware.getInstance();

        robot.rightBack.set(speed);
        robot.rightFront.set(speed);
        robot.leftFront.set(speed);
        robot.leftBack.set(speed);
    }



    public void stop() {
        driveTrain.stop();
    }
}
