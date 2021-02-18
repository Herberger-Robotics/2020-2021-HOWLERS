package org.firstinspires.ftc.teamcode.teleop.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.subsystems.Camera.RingDetector;
import org.firstinspires.ftc.teamcode.subsystems.Scheduler.Commands.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Scheduler.Scheduler;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Scheduler Testing")
public class SchedulerTesting extends HowlersAutoFunction{

    @Override
    public void runOpMode() {

        HowlersHardware robot = HowlersHardware.resetInstance();
        Scheduler scheduler = Scheduler.getInstance();

        initRobot();
        scheduler.schedule(new Drive(Drive.Direction.FORWARD, 1));

        waitForStart();
        while(opModeIsActive()) {
            scheduler.update();
        }
    }
}
