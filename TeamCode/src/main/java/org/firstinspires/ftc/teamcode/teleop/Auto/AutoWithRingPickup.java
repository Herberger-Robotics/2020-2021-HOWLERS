package org.firstinspires.ftc.teamcode.teleop.Auto;

import android.view.accessibility.AccessibilityNodeInfo;

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
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.opencvrepackaged.OpenCvNativeLibCorruptedException;

@Autonomous(name="Howlers Auto w/ Rings")
public class AutoWithRingPickup extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HowlersHardware robot;

    double setPoint = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    OpenCvCamera phoneCamera;
    RingDetector ringDetector;

    public enum State {
        PICK_UP_WOBBLE,
        SPIN_UP_FLYWHEEL,
        POSITION_FIRST,
        SHOOT,
        LOOK_AT_STACK,
        READ_STACK,
        PROCESS_RINGS,
        TURN_FOR_WOBBLE,
        POSITION_WOBBLE,
        DRIVE_TO_WOBBLE,
        DROP_WOBBLE,
        PARK,
        PARK_ZERO,
        END,
    }

    public enum FourRingState {
        START,
        PICK_UP_RINGS,
        FINISHED,
    }
    public FourRingState fourRingState = FourRingState.START;

    public enum OneRingState {
        START,
        SHIFT_LEFT,
        PICK_UP_RINGS,
        SHOOT_RINGS,
        DRIVE_TO_LINE,
        FINISHED,
    }
    public OneRingState oneRingState = OneRingState.START;

    public enum StackHeight {
        ZERO,
        ONE,
        FOUR,
    }

    public StackHeight stackHeight;

    public enum Direction {
        FORWARD,
        LEFT,
        RIGHT,
        BACKWARD,
    }

    public State state = State.PICK_UP_WOBBLE;

    @Override
    public void init() {
        robot = robot.resetInstance();

        robot.init(hardwareMap, true, true, true, true);

        //Wobble Initialization
        robot.wobbleGoal.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Camera Initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.FRONT, cameraMonitorViewId);

        ringDetector = new RingDetector(phoneCamera);

        //Drivetrain Initialization
        robot.driveTrain.setSpeed(0.5);

        // Tell the driver that initialization is complete.

        dashboard.startCameraStream(phoneCamera,0);

        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void start() {
        runtime.reset();

        pickUpWobble();
    }

    @Override
    public void init_loop() {
        packet.put("Ring Height", ringDetector.getHeight());

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void loop() {
        flywheelController();
        telemetryController();

        switch(state) {
            case PICK_UP_WOBBLE: {
                if(!robot.wobbleGoal.busy()) {
                    setPoint = 830;
                    flywheelController();
                    drive(Direction.BACKWARD,1.6);
                    state = State.SPIN_UP_FLYWHEEL;
                }
            } break;
            case SPIN_UP_FLYWHEEL: {
                if(robot.turret.turretPID.getPositionError() <= 30 && !robot.driveTrain.isBusy()) {
                    robot.driveTrain.set(0);
                    robot.driveTrain.setSpeed(0.2);
                    drive(Direction.RIGHT,0.4);
                    state = State.POSITION_FIRST;
                }
            } break;
            case POSITION_FIRST: {
                if(!robot.driveTrain.isBusy()) {
                    feed();
                    intake();
                    state = State.SHOOT;
                }
            }break;
            case SHOOT: {
                if(!robot.feederMotor.busy()) {
                    drive(Direction.BACKWARD,0.3);
                    state = State.LOOK_AT_STACK;
                }
            } break;
            case LOOK_AT_STACK: {
                if(!robot.driveTrain.isBusy()) {
                    robot.driveTrain.set(0);
                    setPoint = 0;
                    switch(ringDetector.getHeight()) {
                        case ZERO: stackHeight = StackHeight.ZERO; break;
                        case ONE: stackHeight = StackHeight.ONE; break;
                        case FOUR: stackHeight = StackHeight.FOUR; break;
                    }
                    packet.put("FINAL Ring Height", stackHeight);
                    state = State.READ_STACK;
                }
            }break;
            case READ_STACK: {
                if(stackHeight != null) {
                    switch(stackHeight) {
                        case ZERO: {
                            encoderDrive(robot.driveTrain.getSpeed(), 0.6, 0.6);
                            state = State.TURN_FOR_WOBBLE;
                        } break;
                        case ONE: {
                            processOneRing();
                            state = State.PROCESS_RINGS;
                        } break;
                        case FOUR: {
                            processFourRings();
                            state = State.PROCESS_RINGS;
                        } break;
                    }

                }
            } break;
            case PROCESS_RINGS: {
                if(stackHeight == StackHeight.ONE) {
                    if(oneRingState == OneRingState.FINISHED) {
                        encoderDrive(robot.driveTrain.getSpeed(), 0.6, 0.6);
                    } else {
                        processOneRing();
                    }
                } else if(stackHeight == StackHeight.FOUR) {
                    if(fourRingState == FourRingState.FINISHED) {
                        encoderDrive(robot.driveTrain.getSpeed(), 0.6, 0.6);
                    } else {
                        processFourRings();
                    }
                }
            } break;
            case TURN_FOR_WOBBLE: {
                if(!robot.driveTrain.isBusy()) {
                    robot.driveTrain.setSpeed(0.5);
                    if(stackHeight == StackHeight.ZERO || stackHeight == StackHeight.FOUR) {
                        drive(Direction.BACKWARD,0.9);
                    }
                    state = State.POSITION_WOBBLE;
                }
            }break;
            case POSITION_WOBBLE: {
                if(!robot.driveTrain.isBusy()) {
                    switch(stackHeight) {
                        case ONE: drive(Direction.LEFT,0.5); break;
                        case FOUR: drive(Direction.LEFT,1.5); break;
                    }
                    state = State.DRIVE_TO_WOBBLE;
                }
            } break;
            case DRIVE_TO_WOBBLE: {
                if(!robot.driveTrain.isBusy()) {
                    robot.driveTrain.set(0);
                    dropWobble();
                    state = State.DROP_WOBBLE;
                }
            }break;
            case DROP_WOBBLE: {
                if(!robot.wobbleGoal.busy()) {
                    switch(stackHeight) {
                        case ZERO: drive(Direction.RIGHT,0.2); state = State.PARK_ZERO; break;
                        case ONE: drive(Direction.RIGHT, 0.3); state = State.PARK; break;
                        case FOUR: drive(Direction.RIGHT,1.2); state = State.PARK; break;
                    }
                }
            }break;
            case PARK: {
                if(!robot.driveTrain.isBusy()) {
                    if(stackHeight == StackHeight.ZERO) {
                        drive(Direction.LEFT,0.5);
                    }
                    state = state.END;
                }
            } break;
            case PARK_ZERO: {
                if(!robot.driveTrain.isBusy()) {
                    drive(Direction.FORWARD,0.5);
                    state = State.PARK;
                }
            } break;
            case END: {
                if(!robot.wobbleGoal.busy() && !robot.driveTrain.isBusy()) {
                    robot.wobbleGoal.set(0);
                    robot.driveTrain.set(0);
                    robot.flywheel.set(0);
                }
            } break;
        }
    }

    public void flywheelController() {

        robot.turret.turretPID.setSetPoint(setPoint);
        robot.flywheel.setVelocity(robot.turret.turretPID.calculate(robot.flywheel.getVelocity()));

    }

    public void telemetryController() {

        if(HowlersHardware.RobotConstants.displayPID == true) {
            packet.put("flywheelSetSpeed", robot.flywheel.get());
            packet.put("PIDCalculation", robot.turret.turretPID.calculate(robot.flywheel.getVelocity()));
            packet.put("PIDPositionError", robot.turret.turretPID.getPositionError());
            packet.put("Current Velocity", robot.flywheel.getVelocity());
            packet.put("Set Point", setPoint);
        }

        packet.put("Ring Height", ringDetector.getHeight());
        packet.put("Autonomous State", state);
        packet.put("One Ring State", oneRingState);
        packet.put("Four ring State", fourRingState);

        dashboard.sendTelemetryPacket(packet);

    }

    public void processOneRing() {
        switch(oneRingState) {
            case START: {
                drive(Direction.RIGHT,0.1);
                intake();
                setPoint = 830;
                oneRingState = OneRingState.SHIFT_LEFT;
            } break;
            case SHIFT_LEFT: {
                if(!robot.driveTrain.isBusy()) {
                    drive(Direction.FORWARD,0.9);
                    intake();
                    oneRingState = OneRingState.PICK_UP_RINGS;
                }
            } break;
            case PICK_UP_RINGS: {
                if(!robot.driveTrain.isBusy()) {
                    drive(Direction.BACKWARD,0.2);
                    feed();
                    intake();
                    oneRingState = OneRingState.SHOOT_RINGS;
                }
            } break;
            case SHOOT_RINGS: {
                if(!robot.driveTrain.isBusy() && !robot.intakeMotor.busy() &&!robot.feederMotor.busy()) {
                    setPoint = 0;
                    drive(Direction.BACKWARD,0.3);
                    oneRingState = OneRingState.DRIVE_TO_LINE;
                }
            } break;
            case DRIVE_TO_LINE: {
                if(!robot.driveTrain.isBusy()) {
                    oneRingState = OneRingState.FINISHED;
                }
            } break;
        }
    }

    public void processFourRings() {

    }

    public void stop() {
        robot.driveTrain.stop();
        robot.turret.stop();
    }

    public void encoderDrive(double speed, double rightRotations, double leftRotations){

        robot.rightFront.setInverted(true);
        robot.leftBack.setInverted(true);

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
                strafeDrive(speed, rotations * -1, rotations * 1);
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
        feederTarget = robot.feederMotor.getEncoderCount() - (int)((4.5) * (1497.325));
        robot.feederMotor.setTarget(feederTarget);
        robot.feederMotor.runToPosition();
        robot.feederMotor.set(Math.abs(0.5));
    }

    public void intake() {
        int intakeTarget;

        intakeTarget = robot.intakeMotor.getEncoderCount() - (int)((4.5) * (1497.325));

        robot.intakeMotor.setTarget(intakeTarget);

        robot.intakeMotor.runToPosition();

        robot.intakeMotor.set(Math.abs(0.5));
    }

    public void pickUpWobble() {
        int wobbleTarget;

        robot.wobbleGoal.setInverted(true);

        wobbleTarget = robot.wobbleGoal.getEncoderCount() - (int)((0.6) * (1497.325));

        robot.wobbleGoal.setTarget(wobbleTarget);

        robot.wobbleGoal.runToPosition();

        robot.wobbleGoal.set(Math.abs(0.5));
    }

    public void dropWobble() {
        int wobbleTarget;

        robot.wobbleGoal.setInverted(false);

        wobbleTarget = robot.wobbleGoal.getEncoderCount() - (int)((0.6) * (1497.325));

        robot.wobbleGoal.setTarget(wobbleTarget);

        robot.wobbleGoal.runToPosition();

        robot.wobbleGoal.set(Math.abs(0.5));
    }
}
