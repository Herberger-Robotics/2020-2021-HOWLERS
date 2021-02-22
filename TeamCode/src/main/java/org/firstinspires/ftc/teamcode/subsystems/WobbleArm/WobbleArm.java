package org.firstinspires.ftc.teamcode.subsystems.WobbleArm;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HowlersMotor;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

public class WobbleArm extends Subsystem {

    public enum WobbleState {
        DOWN,
        UP,
    }

    public WobbleState wobbleState = WobbleState.DOWN;

    public WobbleArm(final HardwareMap hwMap) {
        HowlersHardware robot = HowlersHardware.getInstance();
        robot.wobbleGoal = new HowlersMotor(hwMap, "wobbleMotor", 134.4);
    }

    public void stop() {
        HowlersHardware robot = HowlersHardware.getInstance();
        robot.wobbleGoal.set(0);
    }

    public void pickUpWobble() {
        int wobbleTarget;

        HowlersHardware robot = HowlersHardware.getInstance();

        robot.wobbleGoal.setInverted(false);

        wobbleTarget = robot.wobbleGoal.getEncoderCount() - (int)((0.6) * (1497.325));

        robot.wobbleGoal.setTarget(wobbleTarget);

        robot.wobbleGoal.runToPosition();

        robot.wobbleGoal.set(Math.abs(0.5));
    }

    public void dropWobble() {
        int wobbleTarget;

        HowlersHardware robot = HowlersHardware.getInstance();

        robot.wobbleGoal.setInverted(false);

        wobbleTarget = robot.wobbleGoal.getEncoderCount() - (int)((0.6) * (1497.325));

        robot.wobbleGoal.setTarget(wobbleTarget);

        robot.wobbleGoal.runToPosition();

        robot.wobbleGoal.set(Math.abs(0.5));
    }
}
