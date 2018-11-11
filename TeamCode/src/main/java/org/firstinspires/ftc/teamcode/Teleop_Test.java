package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop_Test")
public class Teleop_Test extends Teleop_Parent {


    @Override
    public void repeat() {
// Get powers
        double forwardPower = -gamepad1.left_stick_y;
        double turnPower = gamepad1.right_stick_x;

        setDrive(forwardPower, turnPower);
    }

    @Override
    public void setup() {
    }

    @Override
    public void begin() {

    }
}
