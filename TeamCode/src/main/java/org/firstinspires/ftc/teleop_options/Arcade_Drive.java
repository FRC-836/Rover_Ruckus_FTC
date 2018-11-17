package org.firstinspires.ftc.teleop_options;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.parent_classes.Teleop_Parent;

@TeleOp(name = "Teleop Arcade")
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

        setArcadeDrive(forwardPower, turnPower);
    }
}