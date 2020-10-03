package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;

public abstract class HowlersAutoFunction extends LinearOpMode {
    HowlersHardware robot = new HowlersHardware();

    public void initRobot(){
        //robot.init(hardwareMap);
        //telemetry.addLine("Initialized");
        //stopAndReset();


    }

    public void stopAndReset(){

        /*
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         */


    }

    public void encoderDrive(double speed, double rightRotations, double leftRotations){

        robot.init(hardwareMap, true, false, false);

        int backrightTarget;
        int backleftTarget;
        int frontrightTarget;
        int frontleftTarget;

        if(opModeIsActive()){
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

            while (opModeIsActive() && robot.leftBack.busy() && robot.leftFront.busy() && robot.rightBack.busy() && robot.rightFront.busy() )
            {

            }

            robot.rightBack.set(0);
            robot.rightFront.set(0);
            robot.leftFront.set(0);
            robot.leftBack.set(0);

            robot.leftFront.runUsingEncoder();
            robot.leftBack.runUsingEncoder();
            robot.rightBack.runUsingEncoder();
            robot.rightFront.runUsingEncoder();
        }



    }








}
