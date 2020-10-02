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
        /*
        robot.init(hardwareMap);
        int backrightTarget;
        int backleftTarget;
        int frontrightTarget;
        int frontleftTarget;

        if(opModeIsActive()){
            frontrightTarget = robot.rightFront.getCurrentPosition() - (int)((rightRotations) * (1497.325));
            frontleftTarget = robot.leftFront.getCurrentPosition() - (int)((leftRotations) * (1497.325));
            backleftTarget = robot.leftBack.getCurrentPosition() - (int)((leftRotations) * (1497.325));
            backrightTarget = robot.rightBack.getCurrentPosition() - (int)((rightRotations) * (1497.325));

            robot.rightFront.setTargetPosition(frontrightTarget);
            robot.leftFront.setTargetPosition(frontleftTarget);
            robot.rightBack.setTargetPosition(backrightTarget);
            robot.leftBack.setTargetPosition(backleftTarget);

            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightFront.setPower(Math.abs(speed));
            robot.rightBack.setPower(Math.abs(speed));
            robot.leftFront.setPower(Math.abs(speed));
            robot.leftBack.setPower(Math.abs(speed));

            while (opModeIsActive() && robot.leftBack.isBusy() && robot.leftFront.isBusy() && robot.rightBack.isBusy() && robot.rightFront.isBusy() )
            {

            }

            robot.rightBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);

            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

         */

    }








}
