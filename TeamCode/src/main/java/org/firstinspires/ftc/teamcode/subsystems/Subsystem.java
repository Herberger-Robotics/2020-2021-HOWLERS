package org.firstinspires.ftc.teamcode.subsystems;

public abstract class Subsystem {

    public enum SubsystemMode {
        DRIVER_CONTROLLED,
        ROBOT_CONTROLLED,
    }

    public SubsystemMode subsystemMode = SubsystemMode.DRIVER_CONTROLLED;
    public boolean busy;
    abstract public void stop();
    public boolean isBusy() { return busy; }
    public void setBusy() {
        busy = true;
    }
    public SubsystemMode getSubsystemMode() {
        return subsystemMode;
    }
    public void manualControl() {
        subsystemMode = SubsystemMode.DRIVER_CONTROLLED;
    }
    public void robotControl() {
        subsystemMode = SubsystemMode.ROBOT_CONTROLLED;
    }
}
