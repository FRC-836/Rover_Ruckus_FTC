package org.firstinspires.ftc.parent_classes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PID_Controller;
import org.firstinspires.ftc.teamcode.TargetDirection;

public abstract class Robot_Parent extends LinearOpMode {

    protected DcMotor backLeftDrive;
    protected DcMotor backRightDrive;
    protected DcMotor frontLeftDrive;
    protected DcMotor frontRightDrive;
    protected DcMotor landingMotor;
    protected Servo teamMarkerServo;

    protected PID_Controller goToTurnPID = new PID_Controller(0.025, 0.0, 0.0);
    protected PID_Controller holdTurnPID = new PID_Controller(0.01332, 0.0, 0.00195);

    private BNO055IMU imu;

    protected final double TEAM_MARKER_SERVO_UP = 1.0;
    protected final double TEAM_MARKER_SERVO_DOWN = -1.0;
    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        backLeftDrive = hardwareMap.get(DcMotor.class, "bld");
        backRightDrive = hardwareMap.get(DcMotor.class, "brd");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frd");
        landingMotor = hardwareMap.get(DcMotor.class, "lm");
        teamMarkerServo = hardwareMap.get(Servo.class, "tms");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        landingMotor.setDirection(DcMotor.Direction.REVERSE);
        teamMarkerServo.setDirection(Servo.Direction.FORWARD);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        landingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        setupImu();

        TargetDirection.setImu(imu);

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

    protected void setIntake(double intakePower) {
        //TODO: need intake
    }

    protected void setArcadeDrive(double forwardPower, double turnPower) {
        backLeftDrive.setPower(forwardPower + turnPower);
        backRightDrive.setPower(forwardPower - turnPower);
        frontLeftDrive.setPower(forwardPower + turnPower);
        frontRightDrive.setPower(forwardPower - turnPower);
    }

    protected void setTankDrive(double leftPower, double rightPower) {
        backLeftDrive.setPower(leftPower);
        backRightDrive.setPower(rightPower);
        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
    }

    protected void dropTeamMarker() {
        teamMarkerServo.setPosition(TEAM_MARKER_SERVO_DOWN);
    }

    protected void retractTeamMarkerServo() {
        teamMarkerServo.setPosition(TEAM_MARKER_SERVO_UP);
    }

    protected void setLandingMotorPower(double power) {
        landingMotor.setPower(power);
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