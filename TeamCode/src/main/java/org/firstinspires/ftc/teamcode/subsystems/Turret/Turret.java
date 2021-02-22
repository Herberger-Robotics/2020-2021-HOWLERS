package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HowlersMotor;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.teleop.HowlersDrive;

public class Turret extends Subsystem {

    private HowlersMotor m_flywheel;
    public PIDFController turretPID;

    public Turret(final HardwareMap hwMap) {
        HowlersHardware robot = HowlersHardware.getInstance();

        robot.flywheel = new HowlersMotor(hwMap, "flywheel", 28);
        robot.flywheel.setInverted(true);

        turretPID = new PIDFController(HowlersHardware.RobotConstants.flywheelP , HowlersHardware.RobotConstants.flywheelI  , HowlersHardware.RobotConstants.flywheelD, HowlersHardware.RobotConstants.flywheelF);
        turretPID.setTolerance(HowlersHardware.RobotConstants.flywheelTOLERANCE);

        m_flywheel = robot.flywheel;
    }

    public void setSpeed(double speed) {
        m_flywheel.set(speed);
    }

    public void stop() {
        m_flywheel.stopMotor();
    }

    public double getCurrentTicks() {
        return m_flywheel.getEncoderCount();
    }

}
