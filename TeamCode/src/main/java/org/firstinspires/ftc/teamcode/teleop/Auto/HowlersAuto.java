package org.firstinspires.ftc.teamcode.teleop.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.subsystems.Camera.RingDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Howlers Auto")
public class HowlersAuto extends HowlersAutoFunction {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    private double inactiveCount = 0;

    @Override
    public void runOpMode() {

        HowlersHardware robot = HowlersHardware.resetInstance();

        initRobot();

        OpenCvCamera phoneCamera;
        RingDetector ringDetector;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        ringDetector = new RingDetector(phoneCamera);

        robot.driveTrain.setSpeed(0.5);

        waitForStart();
        drive(Direction.BACKWARD, 1.5);
        controlFlywheel(700);
        while(robot.turret.turretPID.getPositionError() > 30 && opModeIsActive()) {
            controlFlywheel(700);
        };
        feed();
        controlFlywheel(0);
        inactiveCount = 0;
        while(opModeIsActive() && inactiveCount < 10) {
            controlFlywheel(0);
            if(robot.turret.turretPID.getPositionError() == 0) {
                inactiveCount++;
            }
        };
        intake();
        drive(Direction.LEFT,0.2);
        controlFlywheel(700);
        while(robot.turret.turretPID.getPositionError() > 30 && opModeIsActive()) {
            controlFlywheel(700);
        };
        feed();
        controlFlywheel(0);
        inactiveCount = 0;
        while(opModeIsActive() && inactiveCount < 10) {
            controlFlywheel(0);
            if(robot.turret.turretPID.getPositionError() == 0) {
                inactiveCount++;
            }
        };
        intake();
        drive(Direction.LEFT,0.2);
        controlFlywheel(700);
        while(robot.turret.turretPID.getPositionError() > 30 && opModeIsActive()) {
            controlFlywheel(700);
        };
        feed();
        controlFlywheel(0);
        inactiveCount = 0;
        while(opModeIsActive() && inactiveCount < 10) {
            controlFlywheel(0);
            if(robot.turret.turretPID.getPositionError() == 0) {
                inactiveCount++;
            }
        };



        //drive(Direction.FORWARD, 0.5);
        /*
        switch(ringDetector.getHeight()) {
            case ZERO:
                wobbleToPositionA();
                packet.put("Wobble Position", "A");
                dashboard.sendTelemetryPacket(packet);
                break;
            case ONE:
                wobbleToPositionB();
                packet.put("Wobble Position", "B");
                dashboard.sendTelemetryPacket(packet);
                break;
            case FOUR:
                wobbleToPositionC();
                packet.put("Wobble Position", "C");
                dashboard.sendTelemetryPacket(packet);
                break;
        }
         */


    }
}
