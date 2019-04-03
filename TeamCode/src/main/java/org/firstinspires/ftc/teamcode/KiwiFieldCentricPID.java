package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Kiwi Field Centric PID",group = "A")
public class KiwiFieldCentricPID extends OpMode {

    private static final double THRESHOLD = 0.1;
    private static final double TIME_TO_START = 0.5;
    protected DcMotor frontRight;
    protected DcMotor frontLeft;
    protected DcMotor center;
    private BNO055IMU imu;

    private PID_Controller turnPid = new PID_Controller(0.018,0.0,0.0015);
    private ElapsedTime pidTimer = new ElapsedTime();
    private boolean turnedLastFrame = false;
    private TargetDirection facingDirection;

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

        gamepad1.setJoystickDeadzone(0.1f);
        setupImu(imu);
        TargetDirection.setImu(imu);
        TargetDirection.setCurrentHeading(0.0);
    }

    @Override
    public void start() {
        facingDirection = TargetDirection.makeTargetToRobotsLeft(0.0);
        turnPid.setSetpoint(0.0);
        turnPid.resetPID();
        turnPid.update(0.0);
        turnPid.update(0.0);
        pidTimer.reset();
    }

    @Override
    public void loop() {
        forwardPower = -gamepad1.left_stick_y;
        turnPower = gamepad1.right_stick_x;
        strafePower = gamepad1.left_stick_x;
        setFieldCentricDrive(forwardPower, strafePower, turnPower, TargetDirection.getHeading());
    }

    private void setFieldCentricDrive(double forwardPower, double strafePower, double turnPower, double heading) {
        double hypotenuse = Math.sqrt((forwardPower * forwardPower) + (strafePower * strafePower));
        double offset = Math.toDegrees(Math.atan2(strafePower, forwardPower)) - heading;
        double fP = hypotenuse * Math.cos(Math.toRadians(offset));
        double sP = hypotenuse * Math.sin(Math.toRadians(offset));
        double pidPower = turnPid.update(facingDirection.calculateDistanceFromTarget());
        
        if (Math.abs(turnPower) < THRESHOLD) {
            if (turnedLastFrame)
            {
                turnedLastFrame = false;
                pidTimer.reset();
            }

            if (pidTimer.seconds() > TIME_TO_START) {
                turnPower = pidPower;
                telemetry.addLine("Using PID");
            }
            else {
                facingDirection.setToFieldDirection(TargetDirection.getHeading());
                telemetry.addLine("Waiting for timer");
            }
        } else {
            turnedLastFrame = true;
            facingDirection.setToFieldDirection(TargetDirection.getHeading());
        }
            
        setKiwiDrive(fP, sP, turnPower);
        telemetry.update();
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
