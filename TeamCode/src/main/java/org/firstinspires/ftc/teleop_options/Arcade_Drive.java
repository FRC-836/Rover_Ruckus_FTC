package org.firstinspires.ftc.teleop_options;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.Interfaces.DriveInterface;
import org.firstinspires.ftc.Interfaces.SuspensionDrive;
import org.firstinspires.ftc.State_Machines_12888.Lander_And_Latch;
import org.firstinspires.ftc.State_Machines_12888.Team_Marker;

@TeleOp(name = "Teleop Arcade")
public class Arcade_Drive extends OpMode {

    private Lander_And_Latch landerAndLatchStateMachine;
    private Team_Marker teamMarkerStateMachine;
    private DriveInterface driveInterface = new SuspensionDrive();

    //Team is using Regular/H drive train
    double forwardPower = 0.0;
    double turnPower = 0.0;
    //long lastCheckedTime;

    //original values 0.001 nad 0.004
    private final double DRIVE_POWER_PER_MS_SPEED_UP = 0.003;
    private final double DRIVE_POWER_PER_MS_SPEED_DOWN = 0.005;
    private final double MAX_FORWARD_POWER = 1.0;
    private final double MAX_TURN_POWER = 1.0;

    @Override
    public void init() {
        landerAndLatchStateMachine = new Lander_And_Latch(hardwareMap, telemetry);
        teamMarkerStateMachine = new Team_Marker(hardwareMap);
        driveInterface.init(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        landerAndLatchStateMachine.start();
        teamMarkerStateMachine.start();
        //lastCheckedTime = System.currentTimeMillis();
    }


    @Override
    public void loop() {
        // Get powers
        double goalForwardPower = -gamepad1.left_stick_y;
        double goalTurnPower = gamepad1.right_stick_x;
        double latchingPower;

        //latch
        if (gamepad1.y)
            landerAndLatchStateMachine.raise();
        else if (gamepad1.a)
            landerAndLatchStateMachine.lower();
        else
            landerAndLatchStateMachine.standby();

        //team marker servo
        if (gamepad1.dpad_up)
            teamMarkerStateMachine.dropTeamMarker();

        /*
        updatePowers(goalForwardPower, goalTurnPower);
        setArcadeDrive(forwardPower, turnPower);

        //intake
        if (gamepad1.right_trigger > 0.5)
            setIntakeLifter(1.0);
        else if (gamepad1.left_trigger > 0.5)
            setIntakeLifter(-0.7);
        else
            setIntakeLifter(0.15);

        if (gamepad1.b)
            setMotorIntake(1.0);
        else if (gamepad1.left_bumper)
            setMotorIntake(-1.0);
        else
            setMotorIntake(0.0);
            */
    }

    protected void updatePowers(double goalPowerForward, double goalPowerTurn) {
        /*
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
        */
    }

    @Override
    public void stop() {
        teamMarkerStateMachine.stopThread();
        landerAndLatchStateMachine.stopThread();
    }
}