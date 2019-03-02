package org.firstinspires.ftc.teleop_options;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.parent_classes.Teleop_Parent;

enum State_Enum {
    RAISING,
    LOWERING,
    UNLOCKING,
    STANDBY
}

@TeleOp(name = "Teleop Arcade")
public class Arcade_Drive extends Teleop_Parent {

    //Team is using Regular/H drive train
    double forwardPower = 0.0;
    double turnPower = 0.0;
    long lastCheckedTime;

    private State_Enum currentState = State_Enum.UNLOCKING;
    private ElapsedTime unlockingTimer = new ElapsedTime();
    private ElapsedTime lockingTimer = new ElapsedTime();
    private final double DEPLOY_POWER = 0.4;
    private final double RAISE_POWER = -1.0;
    private final double UNLOCKING_POWER = 1.0; // 1.0
    private final double LOCKED_POWER = -0.2; // 0.4
    private final double LOCKING_POWER = -1.0; // 0.0
    private final double UNLOCKED_POWER = 0.2; // 0.6
    private final double LOCKING_TIME = 750.0;
    private final double UNLOCKING_TIME = 400.0;
    private boolean lastButtonWasUp = false;

    //original values 0.001 nad 0.004
    private final double DRIVE_POWER_PER_MS_SPEED_UP = 0.003;
    private final double DRIVE_POWER_PER_MS_SPEED_DOWN = 0.005;
    private final double MAX_FORWARD_POWER = 1.0;
    private final double MAX_TURN_POWER = 1.0;

    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        lockingTimer.reset();
        unlockingTimer.reset();
        setState(State_Enum.STANDBY);
        lastCheckedTime = System.currentTimeMillis();
    }

    @Override
    public void repeat() {
        if (gamepad1.dpad_down) {
            setArcadeDrive(0.0, 0.0);
            setIntakeLifter(-1.0);
            setInOut(0.0);
            setMotorIntake(0.0);
            setState(State_Enum.LOWERING);
            updateLatchLock();
            return;
        }
        // Get powers
        double goalForwardPower = -gamepad1.left_stick_y;
        double goalTurnPower = gamepad1.right_stick_x;

        //latch
        if (gamepad1.x)
            setState(State_Enum.UNLOCKING);
        else if (gamepad1.b)
            setState(State_Enum.LOWERING);
        else
            setState(State_Enum.STANDBY);
        updateLatchLock();

        //intake extend/retract
        if (gamepad1.y)
            setInOut(1.0);
        else if (gamepad1.a)
            setInOut(-1.0);
        else
            setInOut(0.0);
        updatePowers(goalForwardPower, goalTurnPower);
        setArcadeDrive(forwardPower, turnPower);

        if (gamepad1.right_bumper) {
            setIntakeLifter(0.6);
            lastButtonWasUp = true;
        } else if (gamepad1.right_trigger > 0.5f) {
            setIntakeLifter(-0.4);
            lastButtonWasUp = false;
        } else {
            if (lastButtonWasUp)
                setIntakeLifter(0.01);
            else
                setIntakeLifter(0.0);
        }

        if (gamepad1.left_trigger > 0.5f)
            setMotorIntake(1.0);
        else if (gamepad1.left_bumper)
            setMotorIntake(-1.0);
        else
            setMotorIntake(0.0);
        telemetry.addData("Power: ", intakeLifter.getPower());
    }

    protected void updatePowers(double goalPowerForward, double goalPowerTurn) {
        long newCheckedTime = System.currentTimeMillis();

        double incrementForward = (double) (newCheckedTime - lastCheckedTime);
        double incrementTurn = incrementForward;

        if (Math.abs(goalPowerForward) > Math.abs(forwardPower))
            incrementForward *= DRIVE_POWER_PER_MS_SPEED_UP;
        else
            incrementForward *= DRIVE_POWER_PER_MS_SPEED_DOWN;

        if (Math.abs(goalPowerTurn) > Math.abs(turnPower))
            incrementTurn *= DRIVE_POWER_PER_MS_SPEED_UP;
        else
            incrementTurn *= DRIVE_POWER_PER_MS_SPEED_DOWN;

        if (Math.abs(incrementForward) > Math.abs(forwardPower - goalPowerForward))
            forwardPower = goalPowerForward;
        else if (goalPowerForward > forwardPower)
            forwardPower += incrementForward;
        else
            forwardPower -= incrementForward;

        if (Math.abs(incrementTurn) > Math.abs(turnPower - goalPowerTurn))
            turnPower = goalPowerTurn;
        else if (goalPowerTurn > turnPower)
            turnPower += incrementTurn;
        else
            turnPower -= incrementTurn;

        forwardPower = Range.clip(forwardPower, -MAX_FORWARD_POWER, MAX_FORWARD_POWER);
        turnPower = Range.clip(turnPower, -MAX_TURN_POWER, MAX_TURN_POWER);

        lastCheckedTime = newCheckedTime;
    }

    private void updateLatchLock() {
        switch (currentState) {
            case RAISING:
                landingMotor.setPower(DEPLOY_POWER);
                latchLockServo.setPower(UNLOCKED_POWER);
                break;
            case LOWERING:
                landingMotor.setPower(RAISE_POWER);
                lockServo();
                break;
            case UNLOCKING:
                landingMotor.setPower(0.0);
                latchLockServo.setPower(UNLOCKING_POWER);
                if (unlockingTimer.milliseconds() > UNLOCKING_TIME) {
                    setState(State_Enum.RAISING);
                }
                break;
            case STANDBY:
                landingMotor.setPower(0.0);
                lockServo();
                break;
        }
    }

    private void setState(State_Enum state) {
        if (state == currentState)
            return;
        if (state == State_Enum.UNLOCKING && currentState == State_Enum.RAISING)
            return;

        if (state == State_Enum.UNLOCKING)
            unlockingTimer.reset();
        if (currentState == State_Enum.UNLOCKING || currentState == State_Enum.RAISING)
            lockingTimer.reset();

        currentState = state;
    }

    private void lockServo() {
        if (lockingTimer.milliseconds() < LOCKING_TIME)
            latchLockServo.setPower(LOCKING_POWER);
        else
            latchLockServo.setPower(LOCKED_POWER);
    }

    @Override
    public void end() {

    }
}