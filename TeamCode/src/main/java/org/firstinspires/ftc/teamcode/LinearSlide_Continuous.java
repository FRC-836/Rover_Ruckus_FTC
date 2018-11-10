package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "LinearSlide_Continuous")
public class LinearSlide_Continuous extends OpMode {

    private DcMotor leadScrew;

    private final double POWER = 0.5;

    @Override
    public void init() {
        leadScrew = hardwareMap.get(DcMotor.class, "ls");
        leadScrew.setDirection(DcMotor.Direction.FORWARD);
        leadScrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper)
            leadScrew.setPower(POWER);
        else if (gamepad1.right_trigger > 0.5f)
            leadScrew.setPower(-POWER);
        else
            leadScrew.setPower(0.0);
    }
}
