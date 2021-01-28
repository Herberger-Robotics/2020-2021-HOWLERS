package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HowlersMotor;

public class Intake {

    public Intake(final HardwareMap hwMap) {
        HowlersHardware robot = HowlersHardware.getInstance();
        robot.intake = new HowlersMotor(hwMap, "intakeMotor", 134.4);
    }
}
