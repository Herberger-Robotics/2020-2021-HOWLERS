package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HowlersMotor;

public class Turret {

    private HowlersMotor m_flywheel;

    public Turret(final HardwareMap hwMap) {
        HowlersHardware robot = HowlersHardware.getInstance();
        robot.flywheel = new HowlersMotor(hwMap, "flywheel", 145.6);
        robot.flywheel.setInverted(true);
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
