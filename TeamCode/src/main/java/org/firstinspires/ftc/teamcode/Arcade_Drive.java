package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop Octo")
public class Arcade_Drive extends Teleop_Parent {

    //Team is using Regular/H drive train

    @Override
    public void setup() {

    }

    @Override
    public void begin() {

    }

    @Override
    public void repeat() {
        // Get powers
        double forwardPower = -gamepad1.left_stick_y;
        double turnPower = gamepad1.right_stick_x;

        setDrive(forwardPower, turnPower);
    }
}