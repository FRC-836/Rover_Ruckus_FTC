package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Driver Centric Mode Test", group="Competition")//Competition/Main
public class Driver_Centric_Mode extends OpMode
{
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private BNO055IMU imu = null;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frd");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bld");
        backRightDrive = hardwareMap.get(DcMotor.class, "brd");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        gamepad1.setJoystickDeadzone(0.1f);
        gamepad2.setJoystickDeadzone(0.1f);

        setupImu();
        TargetDirection.setImu(imu);
        TargetDirection.setCurrentHeading(0.0);
    }

    @Override
    public void loop() {
        double leftPower;
        double rightPower;

        double away = -gamepad1.left_stick_y;
        double side  = gamepad1.left_stick_x;
        double turn  =  gamepad1.right_stick_x;
        double directionOfJoystick = Math.toDegrees(Math.atan2(side, away));

        double heading = TargetDirection.getHeading();
        double robotOrientedCommand = directionOfJoystick - heading;

        double inputSize = Math.sqrt(Math.pow(away, 2) + Math.pow(side, 2));
        double strafe = inputSize * Math.sin(Math.toRadians(robotOrientedCommand));
        double forward = inputSize * Math.cos(Math.toRadians(robotOrientedCommand));

        setDrive(forward, turn, strafe);

        telemetry.addData("Direction",directionOfJoystick);
        telemetry.addData("Field Heading",TargetDirection.getHeading());
        telemetry.update();
    }

    private void setDrive(double forwardsPower, double turnPower, double strafePower) {
        frontRightDrive.setPower(forwardsPower - turnPower - strafePower);
        backRightDrive.setPower(forwardsPower - turnPower + strafePower);
        frontLeftDrive.setPower(forwardsPower + turnPower + strafePower);
        backLeftDrive.setPower(forwardsPower + turnPower - strafePower);
    }

    private void setupImu() {
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(imuParameters);
    }
}