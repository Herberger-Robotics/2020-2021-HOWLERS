package org.firstinspires.ftc.teamcode.teleop.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.subsystems.Camera.RingDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="FiniteState")
public class HowlersAutoFiniteState extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HowlersHardware robot;

    double setPoint = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    OpenCvCamera phoneCamera;
    RingDetector ringDetector;

    public enum State {
        SHOOT_FIRST,
        TRANSITION_TO_SECOND,
        SHOOT_SECOND,
        TRANSITION_TO_THIRD,
        SHOOT_THIRD,
        DELIVER_WOBBLE,
        PARK,
    }

    public enum Direction {
        FORWARD,
        LEFT,
        RIGHT,
        BACKWARD,
    }

    public State state = State.SHOOT_FIRST;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = robot.resetInstance();

        robot.init(hardwareMap, true, true, true, true);

        //Wobble Initialization
        robot.wobbleGoal.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        ringDetector = new RingDetector(phoneCamera);

        robot.driveTrain.setSpeed(0.5);

    }

    @Override
    public void start() {
        runtime.reset();

        feed();
        setPoint = 900;
    }

    @Override
    public void loop() {
        flywheelController();

        switch(state) {
            case SHOOT_FIRST: {
                if(!robot.feederMotor.busy()) {
                    setPoint = 0;
                    drive(Direction.LEFT, 0.2);
                    state = State.TRANSITION_TO_SECOND;
                }
            } break;
            case TRANSITION_TO_SECOND: {
                if(!robot.driveTrain.isBusy()) {
                    state = State.SHOOT_SECOND;
                }
            }
        }
    }

    public void flywheelController() {

        robot.turret.turretPID.setSetPoint(setPoint);
        robot.flywheel.setVelocity(robot.turret.turretPID.calculate(robot.flywheel.getVelocity()));
    }

    public void driveTrainController() {

    }

    public void stop() {
        robot.driveTrain.stop();
        robot.turret.stop();
    }

    public void encoderDrive(double speed, double rightRotations, double leftRotations){

        int backrightTarget;
        int backleftTarget;
        int frontrightTarget;
        int frontleftTarget;

        frontrightTarget = robot.rightFront.getEncoderCount() - (int)((rightRotations) * (1497.325));
        frontleftTarget = robot.leftFront.getEncoderCount() - (int)((leftRotations) * (1497.325));
        backleftTarget = robot.leftBack.getEncoderCount() - (int)((leftRotations) * (1497.325));
        backrightTarget = robot.rightBack.getEncoderCount() - (int)((rightRotations) * (1497.325));

        robot.rightFront.setTarget(frontrightTarget);
        robot.leftFront.setTarget(frontleftTarget);
        robot.rightBack.setTarget(backrightTarget);
        robot.leftBack.setTarget(backleftTarget);

        robot.rightBack.runToPosition();
        robot.rightFront.runToPosition();
        robot.leftBack.runToPosition();
        robot.leftFront.runToPosition();

        robot.rightFront.set(Math.abs(speed));
        robot.rightBack.set(Math.abs(speed));
        robot.leftFront.set(Math.abs(speed));
        robot.leftBack.set(Math.abs(speed));
        }

    public void drive(Direction direction, double rotations) {
        double speed = robot.driveTrain.getSpeed();
        switch(direction) {
            case FORWARD:
                encoderDrive(speed, rotations * 1, rotations * -1);
                break;
            case LEFT:
                strafeDrive(speed, rotations * -1, 1);
                break;
            case RIGHT:
                strafeDrive(speed,rotations, rotations * -1);
                break;
            case BACKWARD:
                encoderDrive(speed, rotations * -1, rotations * 1);
                break;
        }

    }

    public void strafeDrive(double speed, double rightRotations, double leftRotations) {

        int backrightTarget;
        int backleftTarget;
        int frontrightTarget;
        int frontleftTarget;

        robot.rightFront.setInverted(false);
        robot.leftBack.setInverted(false);

        frontrightTarget = robot.rightFront.getEncoderCount() - (int)((rightRotations) * (1497.325));
        frontleftTarget = robot.leftFront.getEncoderCount() - (int)((leftRotations) * (1497.325));
        backleftTarget = robot.leftBack.getEncoderCount() - (int)((leftRotations) * (1497.325));
        backrightTarget = robot.rightBack.getEncoderCount() - (int)((rightRotations) * (1497.325));

        robot.rightFront.setTarget(frontrightTarget);
        robot.leftFront.setTarget(frontleftTarget);
        robot.rightBack.setTarget(backrightTarget);
        robot.leftBack.setTarget(backleftTarget);

        robot.rightBack.runToPosition();
        robot.rightFront.runToPosition();
        robot.leftBack.runToPosition();
        robot.leftFront.runToPosition();

        robot.rightFront.set(Math.abs(speed));
        robot.rightBack.set(Math.abs(speed));
        robot.leftFront.set(Math.abs(speed));
        robot.leftBack.set(Math.abs(speed));
    }

    public void feed() {
        int feederTarget;
        feederTarget = robot.feederMotor.getEncoderCount() - (int)((5) * (1497.325));
        robot.feederMotor.setTarget(feederTarget);
        robot.feederMotor.runToPosition();
        robot.feederMotor.set(Math.abs(1));
    }
}
