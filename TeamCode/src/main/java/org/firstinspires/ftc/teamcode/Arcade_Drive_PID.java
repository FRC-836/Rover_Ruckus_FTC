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
        if (gamepad1.dpad_up || gamepad1.dpad_down) {
            if (!dpadPressed) {
                if (gamepad1.dpad_up)
                    p *= 1.1;
                if (gamepad1.dpad_down)
                    p /= 1.1;
                holdTurnPID = new PID_Controller(p, 0.0, d);
                holdTurnPID.setSetpoint(0.0);
                holdTurnPID.resetPID();
            }
            dpadPressed = true;
        } else {
            dpadPressed = false;
        }


        // Get powers
        double forwardPower = -gamepad1.left_stick_y;
        double strafePower = gamepad1.left_stick_x;
        double turnPower = gamepad1.right_stick_x;
        double pidTurnPower = holdTurnPID.update(heading.getRelativeHeading());

        if (Math.abs(strafePower) > JOYSTICK_DEAD_ZONE) {
            if (Math.abs(forwardPower / strafePower) > 1.732)
                strafePower = 0.0;
            if (Math.abs(forwardPower / strafePower) < 0.57735)
                forwardPower = 0.0;
        }

        if (Math.abs(turnPower) < JOYSTICK_DEAD_ZONE && Math.abs(forwardPower) < JOYSTICK_DEAD_ZONE
                && Math.abs(strafePower) < JOYSTICK_DEAD_ZONE) {
            setDrive(0.00, 0.00, 0.00);
            /*if (!isStopped) {

            }
            isStopped = true;
        } else {
            isStopped = false;*/
        } else {
            // If user is trying to turn
            if (Math.abs(turnPower) > JOYSTICK_DEAD_ZONE) {
                // Do what user says
                setDrive(forwardPower, turnPower, strafePower);

                isTurning = true;
            } else // If the user isn't trying to turn
            {
                // Listen to the PID Controller
                if (isTurning) {
                    heading.setRelativeOffset(-Heading.getFieldHeading());
                }

                setDrive(forwardPower, pidTurnPower, strafePower);
                isTurning = false;
            }
        }

        telemetry.addData("pgain", p);
        telemetry.addData("dgain", d);
        telemetry.update();
    }

}