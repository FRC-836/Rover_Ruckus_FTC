package org.firstinspires.ftc.Interfaces;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SuspensionDrive implements DriveInterface {

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;
    private DcMotor backleftDrive;

    @Override
    public void setDrive(double leftPower, double rightPower) {
      frontLeftDrive.setPower(leftPower);
      frontRightDrive.setPower(rightPower);
      backleftDrive.setPower(leftPower);
      backRightDrive.setPower(rightPower);
    }

    @Override
    public void driveDistance(double inches) {

    }

    @Override
    public void turnDegrees(double degrees) {

    }

    @Override
    public boolean isDoneMoving() {
        return false;
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backleftDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }
}
