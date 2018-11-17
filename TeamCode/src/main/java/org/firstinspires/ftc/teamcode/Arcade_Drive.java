package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Arcade Drive", group = "")
public class Arcade_Drive extends Teleop_Parent {
    @Override
    public void begin() {

    }

    @Override
    public void run() {
        double forwardPower = -gamepad1.left_stick_y;
        double turnPower = gamepad1.right_stick_x;
        double strafePower = -gamepad1.left_stick_x;

        if (driveSlowFactor)
        {
            forwardPower *= SLOW_DRIVE_SCALE_FACTOR;
            strafePower *= SLOW_DRIVE_SCALE_FACTOR;
        }


        double extensionPower = 0.0;
        if (gamepad1.dpad_up)
            extensionPower = 1.0;
        else if(gamepad1.dpad_down)
            extensionPower = -1.0;

        double rotationPower = 0.0;
        if (gamepad1.x)
            rotationPower = 1.0;
        else if(gamepad1.a)
            rotationPower= -1.0;

        setDrive(forwardPower, turnPower, strafePower);
        setArmExtender(extensionPower);
        setArmRotator(rotationPower);
    }
}
