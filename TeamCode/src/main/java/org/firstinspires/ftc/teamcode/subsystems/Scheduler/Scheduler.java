package org.firstinspires.ftc.teamcode.subsystems.Scheduler;


import org.firstinspires.ftc.teamcode.subsystems.Scheduler.Commands.Command;

import java.util.List;

public class Scheduler {
    // static variable single_instance of type Singleton
    private static Scheduler instance = null;

    public List<Command> schedule;

    // private constructor restricted to this class itself
    private Scheduler() {

    }

    // static method to create instance of Singleton class
    public static Scheduler getInstance() {
        if (instance == null)
            instance = new Scheduler();

        return instance;
    }

    public static void destroyInstance() {
        instance = null;
    }

    public static Scheduler resetInstance() {
        instance = new Scheduler();
        return instance;
    }

    public void schedule(Command commandToSchedule) {
        schedule.add(commandToSchedule);
    }

    public void update() {
    }
}