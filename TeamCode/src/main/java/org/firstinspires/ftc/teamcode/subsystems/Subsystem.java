package org.firstinspires.ftc.teamcode.subsystems;

public abstract class Subsystem {



    public boolean busy;
    abstract public void stop();
    public boolean isBusy() { return busy; }
    public void setBusy() {
        busy = true;
    }
}
