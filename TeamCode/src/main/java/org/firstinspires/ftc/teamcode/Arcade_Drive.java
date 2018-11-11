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

    }

    @Override
    public void repeat() {
        // Get powers
        double forwardPower = -gamepad1.left_stick_y;
        double turnPower = gamepad1.right_stick_x;
        double pidTurnPower = holdTurnPID.update(heading.getRelativeHeading());
        // If user is trying to turn
        if (Math.abs(turnPower) > JOYSTICK_DEAD_ZONE) {
            setDrive(forwardPower, turnPower);
        }
    }
}