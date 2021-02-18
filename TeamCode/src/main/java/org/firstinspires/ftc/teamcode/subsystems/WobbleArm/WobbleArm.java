package org.firstinspires.ftc.teamcode.subsystems.WobbleArm;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HowlersMotor;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

public class WobbleArm extends Subsystem {

    public WobbleArm(final HardwareMap hwMap) {
        HowlersHardware robot = HowlersHardware.getInstance();
        robot.wobbleGoal = new HowlersMotor(hwMap, "wobbleMotor", 134.4);
    }

    public void stop() {
        HowlersHardware robot = HowlersHardware.getInstance();
        robot.wobbleGoal.set(0);
    }
}
