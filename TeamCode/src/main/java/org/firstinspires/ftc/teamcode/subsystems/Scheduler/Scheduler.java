package org.firstinspires.ftc.teamcode.subsystems.Scheduler;


import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.subsystems.Scheduler.Commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

public class Scheduler {
    // static variable single_instance of type Singleton
    private static Scheduler instance = null;

    public List<Command> commandList = new ArrayList<Command>();

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
        commandList.add(commandToSchedule);
    }

    public void update() {
        HowlersHardware robot = HowlersHardware.getInstance();
        for(int i = 0; i < commandList.size(); i++) {
            Command cmd = commandList.get(i);
            switch(cmd.getStatus()) {
                case IDLE: {
                    List<HowlersHardware.SubsystemType> requiredSubsystems = cmd.requiredSubsystems;
                    boolean subsystemsBusy = false;
                    for(int j = 0; j < requiredSubsystems.size(); j++) {
                        Subsystem requiredSubsystem = robot.getSubsystem(requiredSubsystems.get(j));
                        if(requiredSubsystem.isBusy()) subsystemsBusy = true;
                    }
                    if(!subsystemsBusy) {
                        for(int j = 0; j < requiredSubsystems.size(); j++) {
                            Subsystem requiredSubsystem = robot.getSubsystem(requiredSubsystems.get(j));
                            requiredSubsystem.setBusy();
                        }
                        cmd.start();
                    }
                    break;
                }
                case ACTIVE: cmd.update(); break;
                case FINISHED: commandList.remove(i);
                default: break;
            }
        }
    }
}