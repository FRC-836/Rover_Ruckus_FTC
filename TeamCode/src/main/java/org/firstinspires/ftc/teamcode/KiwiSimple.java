package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Kiwi Simple",group = "A")
public class KiwiSimple extends OpMode {
    protected DcMotor frontRight;
    protected DcMotor frontLeft;
    protected DcMotor center;

    private final double SQRT_3_OVER_2 = 0.86602540378443864676372317075294;

    protected double forwardPower = 0.0;
    protected double strafePower = 0.0;
    protected double turnPower = 0.0;
    @Override
    public void init() {
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        center = hardwareMap.get(DcMotor.class, "ct");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        center.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        center.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gamepad1.setJoystickDeadzone(0.1f);
    }

    @Override
    public void loop() {
        forwardPower = -gamepad1.left_stick_y;
        turnPower = gamepad1.right_stick_x;
        strafePower = gamepad1.left_stick_x;
        setKiwiDrive(forwardPower, strafePower, turnPower);
    }

    private void setKiwiDrive(double forwardPower, double strafePower, double turnPower) {
        frontRight.setPower(-0.5*strafePower + SQRT_3_OVER_2*forwardPower - turnPower);
        frontLeft.setPower(0.5*strafePower + SQRT_3_OVER_2*forwardPower + turnPower);
        center.setPower(strafePower - turnPower);
    }
}
