package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Button Kiwi Drive",group = "A")
public class ButtonKiwiDrive extends OpMode {

    private static final double TURN_MAGNITUDE_THRESHOLD = 0.9;

    protected DcMotor frontRight;
    protected DcMotor frontLeft;
    protected DcMotor center;
    private BNO055IMU imu;

    private TargetDirection target;
    PID_Controller turnPid = new PID_Controller(0.018,0.0,0.0015);

    private final double SQRT_3_OVER_2 = 0.86602540378443864676372317075294;

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

        target = TargetDirection.makeTargetAtFieldPosition(0.0);
        turnPid.setSetpoint(0.0);
        turnPid.resetPID();
        turnPid.update(0.0);
    }

    @Override
    public void start()
    {
        turnPid.update(0.0);
    }

    @Override
    public void loop() {
        double movementUp = -gamepad1.left_stick_y;
        double movementRight = gamepad1.left_stick_x;
        double directionUp = (gamepad1.y ? 1.0 : 0.0) - (gamepad1.a ? 1.0 : 0.0);
        double directionRight = (gamepad1.b ? 1.0 : 0.0) - (gamepad1.x ? 1.0 : 0.0);

        double movementPower = getMagnitude(movementRight, movementUp);
        double movementDirection = getDirection(movementUp, movementRight);
        double robotHeading = TargetDirection.getHeading();
        double robotOrientedDirection = movementDirection - robotHeading;

        double forwardPower = movementPower * Math.cos(Math.toRadians(robotOrientedDirection));
        double strafePower = movementPower * Math.sin(Math.toRadians(robotOrientedDirection));

        if (Math.abs(directionRight) > TURN_MAGNITUDE_THRESHOLD || Math.abs(directionUp) > TURN_MAGNITUDE_THRESHOLD)
            target.setToFieldDirection(getDirection(directionUp, directionRight));

        double distanceFromTarget = target.calculateDistanceFromTarget();

        double turnPower = turnPid.update(distanceFromTarget);

        telemetry.addData("Robot Heading", robotHeading);
        telemetry.addData("Robot Oriented Direction", robotOrientedDirection);
        telemetry.addData("Forward", forwardPower);
        telemetry.addData("Strafe", strafePower);
        telemetry.addData("Turn Distance", distanceFromTarget);
        telemetry.addData("Turn Power", turnPower);

        setKiwiDrive(forwardPower, strafePower, turnPower);
    }

    private double getMagnitude(double x, double y)
    {
        return Math.sqrt(x*x + y*y);
    }

    private double getDirection(double x, double y)
    {
        return Math.toDegrees(Math.atan2(y, x));
    }

    private void setKiwiDrive(double forwardPower, double strafePower, double turnPower) {
        forwardPower = Range.clip(forwardPower, -1.0, 1.0);
        strafePower = Range.clip(strafePower, -1.0, 1.0);
        turnPower = Range.clip(turnPower, -1.0, 1.0);

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
