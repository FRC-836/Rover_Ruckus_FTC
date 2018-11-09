package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Robot_Parent extends LinearOpMode {

    protected DcMotor backLeftDrive;
    protected DcMotor backRightDrive;
    protected DcMotor frontLeftDrive;
    protected DcMotor frontRightDrive;

    protected PID_Controller goToTurnPID = new PID_Controller(0.025, 0.0, 0.0);
    protected PID_Controller holdTurnPID = new PID_Controller(0.0,0.0,0.0);

    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        backLeftDrive = hardwareMap.get(DcMotor.class, "bld");
        backRightDrive = hardwareMap.get(DcMotor.class, "brd");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frd");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setupImu();

        Heading.setImu(imu);
        Heading.setFieldOffset(-Heading.getAbsoluteHeading());

        initialize();
        telemetry.addLine("Initialization Complete");
        telemetry.update();
        waitForStart();
        play();
    }

    public abstract void initialize();
    public abstract void play();
    public abstract void setup();
    public abstract void begin();

    // Functions

    protected void setDrive(double forwardPower, double turnPower) {
        backLeftDrive.setPower(forwardPower + turnPower);
        backRightDrive.setPower(forwardPower + turnPower);
        frontLeftDrive.setPower(forwardPower + turnPower);
        frontRightDrive.setPower(forwardPower + turnPower);
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
