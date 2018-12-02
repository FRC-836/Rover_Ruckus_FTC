package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Robot_Parent extends LinearOpMode {

    protected final double EC_PER_DEGREE_ARM = 10.0;
    protected final double EC_PER_IN_ARM = 104.7;
    protected DcMotor backLeftDrive;
    protected DcMotor backRightDrive;
    protected DcMotor frontLeftDrive;
    protected DcMotor frontRightDrive;
    protected DcMotor armRotator;
    protected DcMotor armExtender;
    protected PID_Controller holdTurnPID = new PID_Controller(0.025, 0.0, 0.0);
    protected PID_Controller armHoldPID = new PID_Controller(0.0,0.0,0.0);
    protected PID_Controller armMovePID = new PID_Controller(0.000165,0.000,0.0000375);
    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        backLeftDrive = hardwareMap.get(DcMotor.class, "bld");
        backRightDrive = hardwareMap.get(DcMotor.class, "brd");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frd");
        armRotator = hardwareMap.get(DcMotor.class, "ar");
        armExtender = hardwareMap.get(DcMotor.class, "ae");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        armRotator.setDirection(DcMotor.Direction.REVERSE);
        armExtender.setDirection(DcMotor.Direction.FORWARD);

        setupImu();

        TargetDirection.setupImu(0.0, imu);

        getReady();

        waitForStart();

        go();

    }

    abstract public void getReady();

    abstract public void go();

    protected void setDrive(double forwardPower, double turnPower, double strafePower) {
        backLeftDrive.setPower(forwardPower + turnPower - strafePower);
        backRightDrive.setPower(forwardPower - turnPower + strafePower);
        frontLeftDrive.setPower(forwardPower + turnPower + strafePower);
        frontRightDrive.setPower(forwardPower - turnPower - strafePower);
    }

    protected void setArmRotator(double turnPower) {
        armRotator.setPower(turnPower);
    }

    protected void setArmExtender(double extendPower) {
        armExtender.setPower(extendPower);
    }

    private void setupImu() {
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
    protected void waitSeconds(double seconds) {
        sleep((long) (seconds * 1000.0));
    }

    protected double getArmPosition() {
        double position = armRotator.getCurrentPosition();
        position /= EC_PER_DEGREE_ARM;
        return position;
    }

    protected double getExtenderPosition() {
        double position = armExtender.getCurrentPosition();
        position /= EC_PER_IN_ARM;
        return position;
    }

    protected void armRotate(double degrees) {

    }

    protected void rotateArmToPosition_Cody(long maxTimeMs) {
        // Ported from arm_pid branch.
        // Assumes we're at a starting position, turning 180 degrees,
        // past the vertical which is assumed to be 110 degrees above the startPoint.
        long endTime = System.currentTimeMillis() + maxTimeMs;
        double startPosition = getArmPosition();
        double resetPoint = startPosition + 110.0;
        boolean repeated = false;

        armMovePID.setSetpoint(startPosition + 180.0);
        armMovePID.resetPID();

        while (opModeIsActive() && System.currentTimeMillis() < maxTimeMs)
        {
            double armPosition = getArmPosition();
            setArmRotator(armMovePID.update(armPosition));
            if (armPosition >= resetPoint && !repeated)
            {
                repeated = true;
                armMovePID.resetPID();
            }
        }
    }
}

