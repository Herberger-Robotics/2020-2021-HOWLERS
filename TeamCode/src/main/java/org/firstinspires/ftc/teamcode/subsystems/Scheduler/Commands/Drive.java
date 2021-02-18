package org.firstinspires.ftc.teamcode.subsystems.Scheduler.Commands;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.teleop.Auto.HowlersAutoFunction;

public class Drive extends Command {

    public enum Direction {
        FORWARD,
        LEFT,
        RIGHT,
        BACKWARD,
    }
    private Direction direction;
    private double rotations;


    public Drive(Direction _direction, double _rotations) {
        require(HowlersHardware.SubsystemType.DRIVE_TRAIN);
        direction = _direction;
        rotations = _rotations;
    }

    public void start() {
        status = Status.INITIALIZING;
        switch(direction) {
            case FORWARD:
                encoderDrive(1, rotations * 1, rotations * -1);
                break;
            case LEFT:
                strafeDrive(1, rotations * -1, 1);
                break;
            case RIGHT:
                strafeDrive(1,rotations, rotations * -1);
                break;
            case BACKWARD:
                encoderDrive(1, rotations * -1, rotations * -1);
                break;
        }
        status = Status.ACTIVE;
    }
    public void update() {
        if(!robot.leftFront.busy() && !robot.rightFront.busy() && !robot.rightBack.busy() && !robot.leftBack.busy()) {
            stop();
        }
    }
    public void stop() {
        robot.rightBack.set(0);
        robot.rightFront.set(0);
        robot.leftFront.set(0);
        robot.leftBack.set(0);

        robot.leftFront.runUsingEncoder();
        robot.leftBack.runUsingEncoder();
        robot.rightBack.runUsingEncoder();
        robot.rightFront.runUsingEncoder();
        status = Status.FINISHED;
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

    public void strafeDrive(double speed, double rightRotations, double leftRotations) {

    };
}
