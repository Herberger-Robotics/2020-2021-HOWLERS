package org.firstinspires.ftc.teamcode.subsystems;

public abstract class subsystem {

    public boolean busy;
    abstract public void stop();
    public boolean isBusy() { return busy; }
}
