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
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.subsystems.Scheduler.Commands.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret.Turret;

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

    double currentVelocity = 0;
    double setPoint = 0;

    boolean aButtonHeld = false;
    boolean triggerHeld = false;


    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public enum AutoShooterState {
        MANUAL,
        ACTIVE,
        TAKE_CONTROL,
        START_FLYWHEEL,
        RELINQUISH_CONTROL,
    }

    public enum DriveMode {
        SLOW,
        FAST,
    }

    public DriveMode driveMode = DriveMode.FAST;

    public AutoShooterState autoShooterState = AutoShooterState.MANUAL;
        /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = robot.resetInstance();

        robot.init(hardwareMap, true, true, true, true);

        //Gamepad Initialization
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);


        //Turret Initialization
        robot.flywheel.setInverted(HowlersHardware.RobotConstants.invertFlywheel);


        //Wobble Initialization
        robot.wobbleGoal.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        wobbleController();
        //shootingController();

        packet.put("Current Velocity", robot.flywheel.getVelocity());
        packet.put("Set Point", setPoint);

        dashboard.sendTelemetryPacket(packet);
    }

    public void flywheelController() {
        currentVelocity = robot.flywheel.getVelocity();

        robot.turret.turretPID.setSetPoint(setPoint);
        if(toolOp.gamepad.left_bumper) {
            setPoint = -40;
        } else {
            if (robot.turret.getSubsystemMode() == Subsystem.SubsystemMode.DRIVER_CONTROLLED) {
                if (driverOp.getButton(GamepadKeys.Button.DPAD_UP) || toolOp.getButton(GamepadKeys.Button.DPAD_UP)) {
                    setPoint = 850;
                } else if (driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT) || toolOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                    setPoint = 830;
                } else if (driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) || toolOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                    setPoint = 750;
                } else if (driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) || toolOp.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                    setPoint = 0;
                }
            }
        }

        robot.flywheel.setVelocity(robot.turret.turretPID.calculate(robot.flywheel.getVelocity()));
    }

    public void driveTrainController() {
        double speed = robot.driveTrain.getSpeed();

        boolean leftBumperState = driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rightBumperState = driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER);

        double leftTrigger = driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if(robot.driveTrain.getSubsystemMode() == Subsystem.SubsystemMode.DRIVER_CONTROLLED) {
            if (leftBumperState) {
                if(!triggerHeld) {
                    switch (driveMode) {
                        case FAST: {
                            robot.driveTrain.setSpeed(0.25);
                            driveMode = DriveMode.SLOW;
                        } break;
                        case SLOW: {
                            robot.driveTrain.setSpeed(0.6);
                            driveMode = DriveMode.FAST;
                        } break;
                    }
                    triggerHeld = true;
                }
            } else {
                triggerHeld = false;
            }

            double strafe = 0;
            double rotation = driverOp.getLeftX() * speed;
            double forward = driverOp.getLeftY() * speed;

            if (leftTrigger > 0.2 && rightTrigger > 0.2) {
                strafe = 0;
            } else if(leftTrigger > 0.2) {
                strafe = leftTrigger *  -1 * speed;
            } else if(rightTrigger > 0.2) {
                strafe = rightTrigger * 1 * speed;
            }


            robot.driveTrain.drive(strafe, forward, rotation);
        }
    }

    public void intakeController() {
        if(robot.intake.getSubsystemMode() == Subsystem.SubsystemMode.DRIVER_CONTROLLED) {
            if(toolOp.gamepad.right_bumper) {
                robot.intakeMotor.set(0.2);
                robot.feederMotor.set(0.2);
            } else {
                if (driverOp.gamepad.a || toolOp.gamepad.a) {
                    robot.intakeMotor.set(-1);
                } else if (driverOp.gamepad.b || toolOp.gamepad.b) {
                    robot.intakeMotor.set(1);
                } else {
                    robot.intakeMotor.set(0);
                }
                if (driverOp.gamepad.y || toolOp.gamepad.y) {
                    robot.feederMotor.set(1);
                } else if (driverOp.gamepad.x || toolOp.gamepad.x) {
                    robot.feederMotor.set(-1);
                } else {
                    robot.feederMotor.set(0);
                }
            }
        }
    }

    public void wobbleController() {
        if(robot.intake.getSubsystemMode() == Subsystem.SubsystemMode.DRIVER_CONTROLLED) {
            if (toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
                robot.wobbleGoal.set(toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.4);
            } else if (toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
                robot.wobbleGoal.set(toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * -0.4);
            } else {
                robot.wobbleGoal.set(0);
            }
        }
    }
    /*
    public void shootingController() {
        if(toolOp.gamepad.a && autoShooterState == AutoShooterState.MANUAL && !aButtonHeld) {
            autoShooterState = AutoShooterState.TAKE_CONTROL;
            aButtonHeld = true;

        } else if(toolOp.gamepad.a && autoShooterState != AutoShooterState.MANUAL && !aButtonHeld) {
            autoShooterState = AutoShooterState.RELINQUISH_CONTROL;
            aButtonHeld = true;
        } else {
            aButtonHeld = false;
        }

        switch(autoShooterState) {
            case TAKE_CONTROL: {
                robot.turret.robotControl();
                robot.intake.robotControl();
                setPoint = 830;

                autoShooterState = AutoShooterState.ACTIVE;
            } break;
            case ACTIVE: {
                if(!robot.intakeMotor.busy() && !robot.feederMotor.busy()) {
                    setPoint = 0;
                    autoShooterState = AutoShooterState.RELINQUISH_CONTROL;
                }
            } break;
            case RELINQUISH_CONTROL: {
                robot.turret.robotControl();
                robot.intake.robotControl();
                robot.intakeMotor.runUsingEncoder();
                robot.feederMotor.runUsingEncoder();
                setPoint = 0;
            }
        }
    } */

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

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.turret.stop();
        robot.driveTrain.stop();
        robot.intake.stop();
        robot.wobbleGoal.set(0);
    }


}