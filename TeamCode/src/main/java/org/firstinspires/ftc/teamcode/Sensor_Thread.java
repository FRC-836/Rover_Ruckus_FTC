package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicLong;

public class Sensor_Thread extends Thread {

    private AtomicLong armRotatorPostion = new AtomicLong(Double.doubleToLongBits(0.0));
    private AtomicLong armRotatorPower = new AtomicLong(Double.doubleToLongBits(0.0));
    private Semaphore armMotorMutex = new Semaphore(1);
    private double armRotatorDrift;

    private DcMotor armRotator;

    Sensor_Thread(DcMotor armRotator) {
        try {
            armMotorMutex.acquire();

            this.armRotator = armRotator;

            double armPos = ArmTargetDirection.getPitch();
            if (armPos < 5.0) // Invalid
                armPos = 50.0;
            armRotatorDrift = armPos - (armRotator.getCurrentPosition() / 9.5);

            armMotorMutex.release();
        } catch (Exception e) {
            armMotorMutex.release();
            e.printStackTrace();
        }
    }

    @Override
    public void run() {
        updateArmRotator();
    }

    public double getArmRotatorPosition()
    {
        return Double.longBitsToDouble(armRotatorPostion.get());
    }

    public void setArmRotatorPower(double armPower)
    {
        armRotatorPower.set(Double.doubleToLongBits(armPower));
    }

    public double getArmRotatorPower()
    {
        return Double.longBitsToDouble(armRotatorPower.get());
    }

    private void updateArmRotator() {
        double armPosition = ArmTargetDirection.getPitch();
        double armPositionEncoder = 0.0;

        try {
            armMotorMutex.acquire();
            armPositionEncoder = (armRotator.getCurrentPosition() / 9.5);
            armRotator.setPower(armRotatorPower.get());
            armMotorMutex.release();
        } catch (Exception e) {
            armMotorMutex.release();
            e.printStackTrace();
        }

        if (armPosition < 5.0)
        { // Invalid IMU Reading
            armRotatorPostion.set(Double.doubleToLongBits(armPositionEncoder + armRotatorDrift));
        }
        else
        { // Valid IMU Reading
            armRotatorDrift = armPosition - armPositionEncoder;
            armRotatorPostion.set(Double.doubleToLongBits(armPosition));
        }
    }
}
