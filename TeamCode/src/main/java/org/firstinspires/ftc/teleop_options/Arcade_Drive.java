package org.firstinspires.ftc.teleop_options;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.State_Machines_12888.Lander_And_Latch_State_Machine;
import org.firstinspires.ftc.parent_classes.Teleop_Parent;

@TeleOp(name = "Teleop Arcade")
public class Arcade_Drive extends Teleop_Parent {

    private Lander_And_Latch_State_Machine landerAndLatchStateMachine;
    private Thread latchLockThread;
    //Team is using Regular/H drive train
    double forwardPower = 0.0;
    double turnPower = 0.0;
    long lastCheckedTime;

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
        landerAndLatchStateMachine = new Lander_And_Latch_State_Machine(hardwareMap, telemetry);
        latchLockThread = new Thread(landerAndLatchStateMachine);
        lastCheckedTime = System.currentTimeMillis();
        latchLockThread.start();
    }


    @Override
    public void repeat() {
        // Get powers
        double goalForwardPower = -gamepad1.left_stick_y;
        double goalTurnPower = gamepad1.right_stick_x;

        //latch
        if (gamepad1.x)
            landerAndLatchStateMachine.raise();
        else if (gamepad1.b)
            landerAndLatchStateMachine.lower();
        else
            landerAndLatchStateMachine.standby();


        //intake extend/retract
        if (gamepad1.y)
            setInOut(1.0);
        else if (gamepad1.a)
            setInOut(-1.0);
        else
            setInOut(0.0);
        updatePowers(goalForwardPower, goalTurnPower);
        setArcadeDrive(forwardPower, turnPower);

        if (gamepad1.right_bumper)
            setIntakeLifter(1.0);
        else if (gamepad1.right_trigger > 0.5f)
            setIntakeLifter(-0.7);
        else
            setIntakeLifter(0.15);

        if (gamepad1.left_bumper)
            setMotorIntake(1.0);
        else if (gamepad1.left_trigger > 0.5f)
            setMotorIntake(-1.0);
        else
            setMotorIntake(0.0);
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

    @Override
    public void end()
    {
        landerAndLatchStateMachine.stopThread();
        try {
            latchLockThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}