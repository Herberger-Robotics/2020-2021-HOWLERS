package org.firstinspires.ftc.teamcode.teleop.Auto;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.subsystems.Camera.RingDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class HowlersRingAuto extends HowlersAutoFunction {

    @Override
    public void runOpMode() {

        HowlersHardware robot = HowlersHardware.resetInstance();

        initRobot();
        waitForStart();

        OpenCvCamera phoneCamera;
        RingDetector ringDetector;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        ringDetector = new RingDetector(phoneCamera);


        switch(ringDetector.getHeight()) {
            case ZERO:  ; break;
            case ONE: ; break;
            case FOUR: ; break;
        }
    }
}
