package org.firstinspires.ftc.parent_classes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PID_Controller;
import org.firstinspires.ftc.teamcode.TargetDirection;

public abstract class Robot_Parent extends LinearOpMode {

    protected DcMotor backLeftDrive;
    protected DcMotor backRightDrive;
    protected DcMotorSimple frontLeftDrive;
    protected DcMotorSimple frontRightDrive;
    protected DcMotor landingMotor;
    protected CRServo latchLockServo;
    protected Servo teamMarkerServo;
    protected CRServo leftIntakeServo;
    protected CRServo rightIntakeServo;
    protected DcMotor intakeMotor;

    private boolean isLocked = true;

    protected PID_Controller goToTurnPID = new PID_Controller(0.025, 0.0, 0.0);

    private BNO055IMU imu;
    protected boolean isAuto;

    protected final double TEAM_MARKER_SERVO_UP = 1.0;
    protected final double TEAM_MARKER_SERVO_DOWN = 0.0;
    private final double AUTO_DRIVE_CAP = 0.5;
    protected final double DEPLOY_POWER = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        backLeftDrive = hardwareMap.get(DcMotor.class, "bld");
        backRightDrive = hardwareMap.get(DcMotor.class, "brd");
        frontLeftDrive = hardwareMap.get(DcMotorSimple.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotorSimple.class, "frd");
        landingMotor = hardwareMap.get(DcMotor.class, "lm");
        teamMarkerServo = hardwareMap.get(Servo.class, "tms");
        latchLockServo = hardwareMap.get(CRServo.class, "ll");
        leftIntakeServo = hardwareMap.get(CRServo.class, "lis");
        rightIntakeServo = hardwareMap.get(CRServo.class, "ris");
        intakeMotor = hardwareMap.get(DcMotor.class, "im");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        landingMotor.setDirection(DcMotor.Direction.REVERSE);
        teamMarkerServo.setDirection(Servo.Direction.FORWARD);
        latchLockServo.setDirection(CRServo.Direction.REVERSE);
        leftIntakeServo.setDirection(CRServo.Direction.FORWARD);
        rightIntakeServo.setDirection(CRServo.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        landingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setupImu();

        TargetDirection.setImu(imu);

        initialize();
        retractTeamMarkerServo();
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

    protected void setServoIntake(double servoIntakePower) {
        leftIntakeServo.setPower(servoIntakePower);
        rightIntakeServo.setPower(servoIntakePower);
    }

    protected void setMotorIntake(double motorIntakePower) {
        intakeMotor.setPower(motorIntakePower);
    }

    protected void setArcadeDrive(double forwardPower, double turnPower) {
        telemetry.addData("Forwards",forwardPower);
        telemetry.addData("Turn",turnPower);
        telemetry.update();

        if (isAuto)
        {
            forwardPower = Range.clip(forwardPower, -AUTO_DRIVE_CAP, AUTO_DRIVE_CAP);
            turnPower = Range.clip(turnPower, -AUTO_DRIVE_CAP, AUTO_DRIVE_CAP);
        }

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
        if (power > 0.0001) {
            // Unlock Lander
            latchLockServo.setPower(0.3);
            isLocked = false;
            sleep(250);
            landingMotor.setPower(power);
        } else if (!isLocked) {
            landingMotor.setPower(power);
            // Lock Lander
            latchLockServo.setPower(-1.0);
            sleep(250);
            latchLockServo.setPower(0.0);
            isLocked = true;
        } else {
            landingMotor.setPower(power);
        }
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