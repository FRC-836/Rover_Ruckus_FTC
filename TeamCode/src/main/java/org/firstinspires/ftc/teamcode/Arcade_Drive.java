package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Arcade_Drive", group = "main")
public class Arcade_Drive extends Teleop_Parent {
    @Override
    public void begin(){}
    @Override
    public void run(){
        double drive = -(gamepad1.left_stick_y);
        double strafe = (gamepad1.left_stick_x);
        double turn = (gamepad1.right_stick_x);

        if (Math.abs(drive) > Math.abs(strafe))
            setDrive(drive, turn, 0.0);
        else
            setDrive(0.0, turn, strafe);
    }
}
