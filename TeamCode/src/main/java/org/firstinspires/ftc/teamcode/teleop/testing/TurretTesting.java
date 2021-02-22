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

package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.subsystems.Turret.Turret;

import java.nio.file.AtomicMoveNotSupportedException;


@TeleOp(name="Turret Testing", group="Iterative Opmode")

public class TurretTesting extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HowlersHardware robot;

    //BasicDrive basicDrive;
    //ManualTurretController manualTurretController;

    GamepadEx driverOp;
    GamepadEx toolOp;

    double currentVelocity = 0;
    double setPoint = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = HowlersHardware.resetInstance();

        robot.init(hardwareMap, false, true, true, false);

        robot.flywheel.setInverted(HowlersHardware.RobotConstants.invertFlywheel);

        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

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
        robot.turret.turretPID.setSetPoint(setPoint);
        robot.turret.turretPID.setTolerance(HowlersHardware.RobotConstants.flywheelTOLERANCE);
        //_turretPID.control(robot.flywheel);
        //CommandScheduler.getInstance().schedule(basicDrive);
        //CommandScheduler.getInstance().schedule(manualTurretController);
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.feederMotor.set(-1);

        if(driverOp.gamepad.a) {
            setPoint = HowlersHardware.RobotConstants.flywheelSETPOINT;
        } else if(driverOp.gamepad.b) {
            setPoint = 0;
        }

        currentVelocity = robot.flywheel.getVelocity();

        robot.turret.turretPID.setSetPoint(setPoint);
        robot.turret.turretPID.setTolerance(HowlersHardware.RobotConstants.flywheelTOLERANCE);
        robot.turret.turretPID.setPIDF(HowlersHardware.RobotConstants.flywheelP , HowlersHardware.RobotConstants.flywheelI  , HowlersHardware.RobotConstants.flywheelD, HowlersHardware.RobotConstants.flywheelF);

        //robot.flywheel.set(1);
        if(HowlersHardware.RobotConstants.SPEED_OVERRIDE > 0) {
            robot.flywheel.set(HowlersHardware.RobotConstants.SPEED_OVERRIDE);
        } else {
                //robot.flywheel.set(1);
                robot.flywheel.setVelocity(robot.turret.turretPID.calculate(robot.flywheel.getVelocity()));

        }

        packet.put("flywheelSetSpeed", robot.flywheel.get());
        packet.put("PIDCalculation", robot.turret.turretPID.calculate(robot.flywheel.getVelocity()));
        packet.put("PIDPositionError", robot.turret.turretPID.getPositionError());
        packet.put("Current Velocity", robot.flywheel.getVelocity());
        packet.put("Set Point", setPoint);

        dashboard.sendTelemetryPacket(packet);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.turret.stop();
        robot.feederMotor.set(0);
    }


}