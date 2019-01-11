package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

public class Sensor_Runnable implements Runnable {

    private int counterRunnable = 0;
    private AtomicInteger counterMain = new AtomicInteger(0);

    private AtomicLong armRotatorPostion = new AtomicLong(Double.doubleToLongBits(0.0));
    private AtomicLong armRotatorPower = new AtomicLong(Double.doubleToLongBits(0.0));
    private Semaphore armMotorMutex = new Semaphore(1);
    private Telemetry telemetry;
    private double armRotatorDrift;

    private DcMotor armRotator;

    Sensor_Runnable(DcMotor armRotator, Telemetry telemetry) {
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
        this.telemetry = telemetry;
    }

    @Override
    public void run() {
        updateArmRotator();

        telemetry.addData("N_Runnable", ++counterRunnable);
        telemetry.addData("N_Main", counterMain);
        telemetry.addData("Arm Position", getArmRotatorPosition());
        telemetry.addData("Arm Motor Power", getArmRotatorPower());
        telemetry.update();
    }

    public void incrementCounter()
    {
        counterMain.incrementAndGet();
    }

    public double getArmRotatorPosition()
    {
        return Double.longBitsToDouble(armRotatorPostion.get());
    }

    public void setArmRotatorPower(double armPower)
    {
        armPower += 0.2 * Math.cos(Math.toRadians(getArmRotatorPosition()));
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
            armRotator.setPower(Double.longBitsToDouble(armRotatorPower.get()));
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
