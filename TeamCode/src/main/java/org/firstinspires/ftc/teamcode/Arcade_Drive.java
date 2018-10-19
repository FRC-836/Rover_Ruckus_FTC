package org.firstinspires.ftc.teamcode;

public class Arcade_Drive extends Teleop_Parent{

    //Team is using Regular/H drive train

    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        holdTurnPID.resetPID();
        holdTurnPID.setSetpoint(getTurnPosition());
    }

    @Override
    public void repeat() {
        // Get powers
        double forwardPower = -gamepad1.left_stick_y;
        double strafePower = gamepad1.left_stick_x;
        double turnPower = gamepad1.right_stick_x;

        double pidTurnPower = holdTurnPID.update(getTurnPosition());

        // If user is trying to turn
        if (Math.abs(turnPower) > JOYSTICK_DEAD_ZONE)
        {
            // Do what user says
            setDrive(forwardPower,turnPower,strafePower);
        }
        else // If the user isn't trying to turn
        {
            // Listen to the PID Controller
            setDrive(forwardPower,pidTurnPower,strafePower);
        }

    }

}
