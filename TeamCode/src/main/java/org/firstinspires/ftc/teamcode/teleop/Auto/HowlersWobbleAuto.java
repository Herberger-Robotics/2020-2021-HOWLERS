package org.firstinspires.ftc.teamcode.teleop.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.subsystems.Camera.RingDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Howlers Wobble Auto")
public class HowlersWobbleAuto extends HowlersAutoFunction {

    @Override
    public void runOpMode() {

        HowlersHardware robot = HowlersHardware.resetInstance();

        initRobot();

        OpenCvCamera phoneCamera;
        RingDetector ringDetector;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        ringDetector = new RingDetector(phoneCamera);

        waitForStart();



        //pickUpWobble();

        switch(ringDetector.getHeight()) {
            case ZERO:  wobbleToPositionA(); telemetry.addData("Wobble Pathing:", "Position A"); break;
            case ONE: wobbleToPositionB(); telemetry.addData("Wobble Pathing:", "Position A"); break;
            case FOUR: wobbleToPositionC(); telemetry.addData("Wobble Pathing:", "Position A"); break;
        }


    }
}
