package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Kiwi Drive",group = "A")
public class KiwiDrive extends OpMode {
    protected DcMotor frontRight;
    protected DcMotor frontLeft;
    protected DcMotor center;
    private BNO055IMU imu;

    private final double SQRT_3_OVER_2 = 0.86602540378443864676372317075294;

    protected double forwardPower = 0.0;
    protected double strafePower = 0.0;
    protected double turnPower = 0.0;
    @Override
    public void init() {
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        center = hardwareMap.get(DcMotor.class, "ct");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        center.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        center.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setupImu(imu);
        TargetDirection.setImu(imu);
        TargetDirection.setCurrentHeading(0.0);
    }

    @Override
    public void loop() {
        forwardPower = -gamepad1.left_stick_y;
        turnPower = gamepad1.right_stick_x;
        strafePower = gamepad1.left_stick_x;
        setFieldCentricDrive(forwardPower, strafePower, turnPower, TargetDirection.getHeading());
    }

    private void setFieldCentricDrive(double forwardPower, double strafePower, double turnPower, double heading) {
        // TODO: Change forward, strafe, and turn powers based on heading.
        setKiwiDrive(forwardPower, strafePower, turnPower);
    }

    private void setKiwiDrive(double forwardPower, double strafePower, double turnPower) {
        frontRight.setPower(-0.5*strafePower + SQRT_3_OVER_2*forwardPower - turnPower);
        frontLeft.setPower(0.5*strafePower + SQRT_3_OVER_2*forwardPower + turnPower);
        center.setPower(strafePower - turnPower);
    }
    private void setupImu(BNO055IMU imu) {
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imuParameters.temperatureUnit = BNO055IMU.TempUnit.FARENHEIT;

        imu.initialize(imuParameters);

    }
}
