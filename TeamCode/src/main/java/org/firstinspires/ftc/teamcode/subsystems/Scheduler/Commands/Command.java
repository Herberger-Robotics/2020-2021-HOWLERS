package org.firstinspires.ftc.teamcode.subsystems.Scheduler.Commands;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

public abstract class Command {

    public enum Status {
        IDLE,
        INITIALIZING,
        ACTIVE,
        FINISHED,
    }

    public List<HowlersHardware.SubsystemType> requiredSubsystems = new ArrayList<HowlersHardware.SubsystemType>();
    public boolean finished;
    public Status status;

    HowlersHardware robot = HowlersHardware.getInstance();

    abstract public void start();
    abstract public void update();
    abstract public void stop();
    public List<HowlersHardware.SubsystemType> getAffectedSubsystems() {
        return requiredSubsystems;
    }
    public boolean isFinished() {
        return finished;
    }
    public void require(HowlersHardware.SubsystemType requiredSubsystem) {
        requiredSubsystems.add(requiredSubsystem);
    }
    public Status getStatus() {
        return status;
    }
}
