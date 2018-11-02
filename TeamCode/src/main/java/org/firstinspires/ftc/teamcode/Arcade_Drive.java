package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop Octo")
public class Arcade_Drive extends Teleop_Parent {

    //Team is using Regular/H drive train

    Heading heading;

    @Override
    public void setup() {

    }

    @Override
    public void begin() {

        heading = Heading.createRelativeHeading(0.0f);
        holdTurnPID.resetPID();
        holdTurnPID.setSetpoint(0.0);

    }

    @Override
    public void repeat() {
        // Get powers
        double forwardPower = -gamepad1.left_stick_y;
        double strafePower = gamepad1.left_stick_x;
        double turnPower = gamepad1.right_stick_x;
        double pidTurnPower = holdTurnPID.update(heading.getRelativeHeading());
        // If user is trying to turn
        if (Math.abs(turnPower) > JOYSTICK_DEAD_ZONE) {
            // Do what user says
            setDrive(forwardPower, turnPower, strafePower);
        } else // If the user isn't trying to turn
        {
            // Listen to the PID Controller
            setDrive(forwardPower, pidTurnPower, strafePower);
        }
        setDrive(forwardPower, turnPower, strafePower);
    }
}
