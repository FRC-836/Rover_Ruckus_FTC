package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tank_Drive", group = "main")
public class Tank_Drive extends Teleop_Parent {
    @Override
    public void begin(){}
    @Override
    public void run(){
        double leftPower;
        double rightPower;

        leftPower = -mapJoyStick(gamepad1.left_stick_y);
        rightPower = -mapJoyStick(gamepad1.right_stick_y);

        setDrive(leftPower, rightPower);
    }
}
