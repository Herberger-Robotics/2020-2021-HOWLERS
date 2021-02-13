package org.firstinspires.ftc.teamcode.subsystems.WobbleArm;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HowlersMotor;
import org.firstinspires.ftc.teamcode.subsystems.subsystem;

public class WobbleArm extends subsystem {

    public WobbleArm(final HardwareMap hwMap) {
        HowlersHardware robot = HowlersHardware.getInstance();
        robot.wobbleGoal = new HowlersMotor(hwMap, "intakeMotor", 134.4);
    }

    public void stop() {
        HowlersHardware robot = HowlersHardware.getInstance();
        robot.wobbleGoal.set(0);
    }
}
