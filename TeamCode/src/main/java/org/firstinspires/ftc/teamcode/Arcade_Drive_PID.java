package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop Octo PID")
public class Arcade_Drive_PID extends Teleop_Parent {

    //Team is using Regular/H drive train
    Heading heading;
    boolean isTurning = false;
    boolean dpadPressed = false;
    boolean isStopped = false;

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

        if (Math.abs(forwardPower) > Math.abs(strafePower))
            strafePower = 0.0;
        else
            forwardPower = 0.0;

        if (Math.abs(turnPower) < JOYSTICK_DEAD_ZONE && Math.abs(forwardPower) < JOYSTICK_DEAD_ZONE
                && Math.abs(strafePower) < JOYSTICK_DEAD_ZONE) {
            setDrive(0.00, 0.00);
            /*if (!isStopped) {

            }
            isStopped = true;
        } else {
            isStopped = false;*/
        } else {
            // If user is trying to turn
            if (Math.abs(turnPower) > JOYSTICK_DEAD_ZONE) {
                // Do what user says
                setDrive(forwardPower, turnPower);

                isTurning = true;
            } else // If the user isn't trying to turn
            {
                // Listen to the PID Controller
                if (isTurning) {
                    heading.setRelativeOffset(-Heading.getFieldHeading());
                }

                double turnMult = (holdTurnMultiplier - 1.0) * Math.max(Math.abs(forwardPower), Math.abs(strafePower)) + 1.0;

                setDrive(forwardPower, pidTurnPower * turnMult);
                isTurning = false;
            }
        }
    }
}