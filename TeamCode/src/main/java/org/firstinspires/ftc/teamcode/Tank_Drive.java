package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop_Tank")

public class Tank_Drive extends Teleop_Parent {

    @Override
    public void setup() {

    }

    @Override
    public void begin() {

    }

    @Override
    public void repeat() {
        double rightDrive = -gamepad1.right_stick_y;
        double leftDrive = -gamepad1.left_stick_y;

        setTankDrive(leftDrive, rightDrive);
    }
}
