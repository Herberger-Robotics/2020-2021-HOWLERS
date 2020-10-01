package org.firstinspires.ftc.teamcode.hardwaremaps.motors;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HowlersMotor implements Motor {

    private DcMotor m_motor;
    private double resetVal;

    public static double TICKS_PER_REV;

    public HowlersMotor(HardwareMap hMap, String name, double TPR) {
        m_motor = hMap.get(DcMotor.class, name);
        TICKS_PER_REV = TPR;
    }

    @Override
    public void set(double speed) {
        m_motor.setPower(speed);
    }

    @Override
    public double get() {
        return m_motor.getPower();
    }

    @Override
    public void setInverted(boolean isInverted) {
        m_motor.setDirection(!isInverted ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
    }

    @Override
    public boolean getInverted() {
        return m_motor.getDirection() == DcMotor.Direction.REVERSE;
    }

    @Override
    public void disable() {
        m_motor.close();
    }

    @Override
    public String getDeviceType() {
        return null;
    }

    @Override
    public void pidWrite(double output) {
        set(output);
    }

    @Override
    public void stopMotor() {
        set(0);
    }

    public double getEncoderCount() {
        return m_motor.getCurrentPosition() - resetVal;
    }

    public void resetEncoder() {
        resetVal += getEncoderCount();
    }

    public double getNumRevolutions() {
        return getEncoderCount() / TICKS_PER_REV;
    }

}