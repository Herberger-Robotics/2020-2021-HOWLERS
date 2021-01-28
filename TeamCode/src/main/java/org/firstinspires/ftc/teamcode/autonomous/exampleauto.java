package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.teleop.Auto.HowlersAutoFunction;

public class exampleauto extends HowlersAutoFunction {
    HowlersHardware robot;


    @Override
    public void runOpMode(){
        robot = HowlersHardware.resetInstance();
        initRobot();
        waitForStart();
        encoderDrive(0.5,1,1);




    }
}
