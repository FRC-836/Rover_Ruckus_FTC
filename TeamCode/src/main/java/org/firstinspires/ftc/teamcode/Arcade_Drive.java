package org.firstinspires.ftc.teamcode;

public class Arcade_Drive extends Teleop_Parent {
    @Override
    public void begin() {

    }

    @Override
    public void run() {
        double forwardPower = -gamepad1.left_stick_y;
        double turnPower = gamepad1.right_stick_x;
        double strafePower = gamepad1.left_stick_x;

        setDrive(forwardPower, turnPower, strafePower);
    }
}
