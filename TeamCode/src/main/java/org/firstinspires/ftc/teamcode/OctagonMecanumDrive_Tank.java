package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Octagon/Mecanum Drive | Tank", group = "Drive")
public class OctagonMecanumDrive_Tank extends OpMode {

    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;

    @Override
    public void init() {
        backLeftDrive = hardwareMap.get(DcMotor.class, "bld");
        backRightDrive = hardwareMap.get(DcMotor.class, "brd");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frd");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gamepad1.setJoystickDeadzone(0.1f);

        telemetry.addLine("Initialized.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double leftForward = -gamepad1.left_stick_y;
        double leftStrafe = gamepad1.left_stick_x;
        double rightForward = -gamepad1.right_stick_y;
        double rightStrafe = gamepad1.right_stick_x;

        backLeftDrive.setPower(leftForward - leftStrafe);
        backRightDrive.setPower(rightForward + rightStrafe);
        frontLeftDrive.setPower(leftForward + leftStrafe);
        frontRightDrive.setPower(rightForward - rightStrafe);
    }
}
