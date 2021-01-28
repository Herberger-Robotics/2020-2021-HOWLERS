/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.teleop.testing.TurretTesting;


@TeleOp(name="HowlersDrive", group="Iterative Opmode")

public class HowlersDrive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HowlersHardware robot;

    //BasicDrive basicDrive;
    //ManualTurretController manualTurretController;

    GamepadEx driverOp = null;
    GamepadEx toolOp = null;

    private PIDFController _turretPID;
    private double _setpoint;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Config
    public static class RobotConstants {
        public static double flywheelP = 10;
        public static double flywheelI = 0;
        public static double flywheelD = 0;
        public static double flywheelF = 0;
        public static double SPEED_OVERRIDE = 0;
        public static double flywheelSETPOINT = 0;
        public static double flywheelTOLERANCE = 0.01;
        public static boolean invertFlywheel = true;
    }


        /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = robot.resetInstance();

        robot.init(hardwareMap, true, true, true);

        //Gamepad Initialization
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);


        //Turret Initialization
        robot.flywheel.setInverted(TurretTesting.RobotConstants.invertFlywheel);
        _turretPID = new PIDFController(TurretTesting.RobotConstants.flywheelP , TurretTesting.RobotConstants.flywheelI  , TurretTesting.RobotConstants.flywheelD, TurretTesting.RobotConstants.flywheelF);
        _setpoint = RobotConstants.flywheelSETPOINT;

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        //CommandScheduler.getInstance().schedule(basicDrive);
        //CommandScheduler.getInstance().schedule(manualTurretController);
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        driveTrainController();
        flywheelController();
        intakeController();

        packet.put("flywheelSetSpeed", robot.flywheel.get());
        packet.put("PIDCalculation", _turretPID.calculate(robot.flywheel.getVelocity()));
        packet.put("PIDPositionError", _turretPID.getPositionError());
        packet.put("Current Velocity", robot.flywheel.getVelocity());

        dashboard.sendTelemetryPacket(packet);
    }

    public void flywheelController() {

        if(driverOp.gamepad.a) {
            _setpoint = 1000;
        } else if(driverOp.gamepad.b) {
            _setpoint = 0;
        }

        _turretPID.setSetPoint(_setpoint);
        _turretPID.setTolerance(TurretTesting.RobotConstants.flywheelTOLERANCE);
        _turretPID.setPIDF(TurretTesting.RobotConstants.flywheelP , TurretTesting.RobotConstants.flywheelI  , TurretTesting.RobotConstants.flywheelD, TurretTesting.RobotConstants.flywheelF);

        //robot.flywheel.set(1);
        if(TurretTesting.RobotConstants.SPEED_OVERRIDE > 0) {
            robot.flywheel.set(TurretTesting.RobotConstants.SPEED_OVERRIDE);
        } else {
            //robot.flywheel.set(1);
            robot.flywheel.setVelocity(_turretPID.calculate(robot.flywheel.getVelocity()));

        }
    }

    public void driveTrainController() {
        double speed = 0.5;
        double rotation = driverOp.getLeftX() * speed;
        double forward = driverOp.getLeftY() * speed;
        double strafe = 0;

        boolean leftBumperState = driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rightBumperState = driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER);

        if(leftBumperState && rightBumperState) strafe = 0;
        else if (rightBumperState) strafe = 1 * speed;
        else if (leftBumperState) strafe = -1 * speed;

        robot.driveTrain.drive(strafe, forward, rotation);
    }

    public void intakeController() {
        if (driverOp.gamepad.x) {
            robot.intake.set(-1);
        } else if(driverOp.gamepad.y) {
            robot.intake.set(1);
        } else {
            robot.intake.set(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.turret.stop();
        robot.driveTrain.stop();
        robot.intake.set(0);
    }


}