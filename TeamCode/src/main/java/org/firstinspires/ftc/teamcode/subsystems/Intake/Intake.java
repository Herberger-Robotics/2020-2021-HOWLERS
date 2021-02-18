package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HowlersMotor;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

public class Intake extends Subsystem {
    public boolean busy;

    public Intake(final HardwareMap hwMap) {
        HowlersHardware robot = HowlersHardware.getInstance();
        robot.intakeMotor = new HowlersMotor(hwMap, "intakeMotor", 134.4);
        robot.feederMotor = new HowlersMotor(hwMap, "feederMotor", 134.4);
    }

    public void stop() {
        HowlersHardware robot = HowlersHardware.getInstance();
        robot.intakeMotor.set(0);
        robot.feederMotor.set(0);
    }
}
