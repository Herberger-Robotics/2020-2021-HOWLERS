package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;

public class exampleauto extends HowlersAutoFunction {
    HowlersHardware robot = new HowlersHardware();


    @Override
    public void runOpMode(){
        initRobot();
        waitForStart();
        encoderDrive(0.5,1,1);




    }
}
