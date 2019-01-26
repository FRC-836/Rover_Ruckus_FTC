package org.firstinspires.ftc.State_Machines_12888;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicBoolean;

enum State_Enum {
    RAISING,
    LOWERING,
    UNLOCKING,
    STANDBY
}

public class Lander_And_Latch_State_Machine implements Runnable {

    private CRServo latchLockServo;
    private DcMotor landingMotor;
    private State_Enum currentState;
    private ElapsedTime timer = new ElapsedTime();
    private AtomicBoolean opModeIsActive = new AtomicBoolean(true);
    private final double DEPLOY_POWER = 0.4;

    private Semaphore stateLock = new Semaphore(1);

    public Lander_And_Latch_State_Machine(HardwareMap hardwareMap) {
        latchLockServo = hardwareMap.get(CRServo.class, "ll");
        landingMotor = hardwareMap.get(DcMotor.class, "lm");

        latchLockServo.setDirection(CRServo.Direction.FORWARD);
        landingMotor.setDirection(DcMotor.Direction.REVERSE);

        landingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setState(State_Enum.STANDBY);
    }

    @Override
    public void run() {
        while (opModeIsActive.get()) {
            switch (getState())
            {
                case RAISING:
                    landingMotor.setPower(DEPLOY_POWER);
                    latchLockServo.setPower(-0.1);
                    break;
                case LOWERING:
                    landingMotor.setPower(-DEPLOY_POWER);
                    latchLockServo.setPower(0.1);
                    break;
                case UNLOCKING:
                    //TODO: give latchlockservo higher power for the unlocking portion of time
                    landingMotor.setPower(0.0);
                    latchLockServo.setPower(-0.1);
                    if (timer.milliseconds() > 500.0) {
                        setState(State_Enum.RAISING);
                    }
                    break;
                case STANDBY:
                    latchLockServo.setPower(0.1);
                    landingMotor.setPower(0.0);
                    break;
            }
            sleep(15);
        }
    }

    private synchronized State_Enum getState()
    {
        while (!stateLock.tryAcquire())
        {
            sleep(0);
        }
        State_Enum state = currentState;
        stateLock.release();

        return state;
    }

    private synchronized void setState(State_Enum state)
    {
        while (!stateLock.tryAcquire())
        {
            sleep(0);
        }
        currentState = state;
        stateLock.release();
    }

    public void raise() {
        if (getState() == State_Enum.STANDBY || getState() == State_Enum.LOWERING)
        {
            setState(State_Enum.UNLOCKING);
            timer.reset(); // TODO: Make timer access thread-safe
        }
    }

    public void lower() {
        setState(State_Enum.LOWERING);
    }

    public void stopLatch() {
        setState(State_Enum.STANDBY);
    }

    public void stopThread() {
        opModeIsActive.set(false);
    }

    private final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
/*
abstract class State_Class {
}

class Raising extends State_Class {
}

class Lowering extends State_Class {
}

class Unlocking extends State_Class {
}

class Standby extends State_Class {
}

*/