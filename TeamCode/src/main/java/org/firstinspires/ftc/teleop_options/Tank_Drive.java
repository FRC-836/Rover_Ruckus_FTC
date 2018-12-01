package org.firstinspires.ftc.teleop_options;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.parent_classes.Teleop_Parent;

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
        double latchingPower;

        if (gamepad1.right_trigger > 0.5f)
            latchingPower = 1.0;
        else if (gamepad1.left_trigger > 0.5f)
            latchingPower = -1.0;
        else
            latchingPower = 0.0;

        setTankDrive(leftDrive, rightDrive);
        setLandingMotorPower(latchingPower);
    }
}
