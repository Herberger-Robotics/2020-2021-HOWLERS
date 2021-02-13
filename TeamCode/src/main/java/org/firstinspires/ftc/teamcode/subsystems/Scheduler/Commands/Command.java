package org.firstinspires.ftc.teamcode.subsystems.Scheduler.Commands;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;

import java.util.List;

public abstract class Command {

    public enum subsystem {
        DRIVE_TRAIN,
        CAMERA,
        INTAKE,
        TURRET,
    }

    public List<subsystem> requiredSubsystems;
    public boolean finished;

    HowlersHardware robot = HowlersHardware.getInstance();

    abstract public void start();
    abstract public void update();
    abstract public void stop();
    public List<subsystem> getAffectedSubsystems() {
        return requiredSubsystems;
    }
    public boolean isFinished() {
        return finished;
    }
    public void require(subsystem requiredSubsystem) {
        requiredSubsystems.add(requiredSubsystem);
    }
}
