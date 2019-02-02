package org.firstinspires.ftc.State_Machines_12888;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    private State_Enum currentState = State_Enum.UNLOCKING;
    private ElapsedTime unlockingTimer = new ElapsedTime();
    private ElapsedTime lockingTimer = new ElapsedTime();
    private AtomicBoolean opModeIsActive = new AtomicBoolean(true);
    private final double DEPLOY_POWER = 0.4;

    private final double UNLOCKING_POWER = 1.0;
    private final double LOCKED_POWER = 0.4;
    private final double LOCKING_POWER = 0.0;
    private final double UNLOCKED_POWER = 0.60;

    private final double LOCKING_TIME = 500.0;
    private final double UNLOCKING_TIME = 500.0;

    private Telemetry telemetry;

    private Semaphore stateAndTimerLock = new Semaphore(1);
    private Semaphore unlockingTimerLock = new Semaphore(1);

    public Lander_And_Latch_State_Machine(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        latchLockServo = hardwareMap.get(CRServo.class, "ll");
        landingMotor = hardwareMap.get(DcMotor.class, "lm");

        latchLockServo.setDirection(CRServo.Direction.FORWARD);
        landingMotor.setDirection(DcMotor.Direction.REVERSE);

        landingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setState(State_Enum.STANDBY);

        telemetry.addLine("Thread Created");
        telemetry.update();
    }

    @Override
    public void run() {
        telemetry.addLine("Thread Started");
        telemetry.update();
        String state = "None";
        while (opModeIsActive.get()) {
            switch (getState()) {
                case RAISING:
                    state = "Raising";
                    landingMotor.setPower(DEPLOY_POWER);
                    latchLockServo.setPower(UNLOCKED_POWER);
                    break;
                case LOWERING:
                    state = "Lowering";
                    landingMotor.setPower(-DEPLOY_POWER);
                    lockServo();
                    break;
                case UNLOCKING:
                    state = "Unlocking";
                    landingMotor.setPower(0.0);
                    latchLockServo.setPower(UNLOCKING_POWER);
                    while (!unlockingTimerLock.tryAcquire()) {
                        sleep(0);
                    }
                    if (unlockingTimer.milliseconds() > UNLOCKING_TIME) {
                        setState(State_Enum.RAISING);
                    }
                    unlockingTimerLock.release();
                    break;
                case STANDBY:
                    state = "Standby";
                    landingMotor.setPower(0.0);
                    lockServo();
                    break;
            }
            telemetry.addLine(state);
            telemetry.addData("Latch Power", latchLockServo.getPower());
            telemetry.addData("Lander Power", landingMotor.getPower());
            telemetry.update();
            sleep(15);
        }
    }

    private synchronized State_Enum getState() {
        while (!stateAndTimerLock.tryAcquire()) {
            sleep(0);
        }
        State_Enum state = currentState;
        stateAndTimerLock.release();

        return state;
    }

    private synchronized void setState(State_Enum state) {
        while (!stateAndTimerLock.tryAcquire()) {
            sleep(0);
        }

        if (state == currentState)
            return;
        if (state == State_Enum.UNLOCKING && currentState == State_Enum.RAISING)
            return;

        if (state == State_Enum.UNLOCKING)
            unlockingTimer.reset();
        if (currentState == State_Enum.UNLOCKING || currentState == State_Enum.RAISING)
            lockingTimer.reset();

        currentState = state;
        stateAndTimerLock.release();
    }

    public void raise() {
        setState(State_Enum.UNLOCKING);
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

    private synchronized void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void lockServo() {
        while (!stateAndTimerLock.tryAcquire()) {
            sleep(0);
        }
        if (lockingTimer.milliseconds() < LOCKING_TIME)
            latchLockServo.setPower(LOCKING_POWER);
        else
            latchLockServo.setPower(LOCKED_POWER);
        stateAndTimerLock.release();
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