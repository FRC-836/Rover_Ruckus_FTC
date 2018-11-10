package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "LinearSlide_Discrete")
public class LinearSlide_Discrete extends OpMode {

    private final int INCREMENT = 200;
    private final double P_GAIN = 0.005;

    DcMotor leadScrew;
    int goalPos = 0;
    boolean buttonsEnabled = true;

    @Override
    public void init() {
        leadScrew = hardwareMap.get(DcMotor.class, "ls");
        leadScrew.setDirection(DcMotor.Direction.FORWARD);
        leadScrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        goalPos = leadScrew.getCurrentPosition();
    }

    @Override
    public void loop() {
        boolean up = gamepad1.right_bumper;
        boolean down = gamepad1.right_trigger > 0.5f;
        if (up || down)
        {
            if (buttonsEnabled)
            {
                if (up)
                    goalPos += INCREMENT;
                if (down)
                    goalPos -= INCREMENT;
                buttonsEnabled = false;
            }
        }
        else {
            buttonsEnabled = true;
        }

        leadScrew.setPower(P_GAIN * (goalPos - leadScrew.getCurrentPosition()));
    }
}
