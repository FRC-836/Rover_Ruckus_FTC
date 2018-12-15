package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Lander_Control", group = "main")
public class Lander_Control extends Teleop_Parent {
    @Override
    public void begin() {

    }

    @Override
    public void run() {
        setArmLander(-gamepad1.right_stick_y);
        telemetry.addData("Encoder", getArmLanderPosition());
        telemetry.update();
    }
}
