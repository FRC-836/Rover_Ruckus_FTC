package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public abstract class Robot_Parent extends LinearOpMode {
    //Set up robot parts/controllers
    protected final double EC_PER_DEGREE_ARM = 20.04;
    protected final double EC_PER_IN_ARM = 104.7;
    protected DcMotor backLeftDrive;
    protected DcMotor backRightDrive;
    protected DcMotor frontLeftDrive;
    protected DcMotor frontRightDrive;
    protected DcMotor armRotator;
    protected DcMotor armExtender;
    protected DcMotor armLander;
    protected Servo markerReleaser;
    private BNO055IMU imu;
    protected DcMotor intakeMotor;
    protected boolean isMovingToGoal = false;

    private boolean isArmHolding = false;

    public double pStableHoldTurn = 0.019;
    public double dStableHoldTurn = 0.00195;
    public double holdTurnMultiplier = 5.25;

    protected PID_Controller holdTurnPID = new PID_Controller(pStableHoldTurn, 0.0, dStableHoldTurn);
    protected PID_Controller armHoldPID = new PID_Controller(0.000165, 0.0, 0.000375);

    //Maps robot parts to data values in config file, sets up opMode
    @Override
    public void runOpMode() throws InterruptedException {
        backLeftDrive = hardwareMap.get(DcMotor.class, "bld");
        backRightDrive = hardwareMap.get(DcMotor.class, "brd");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frd");
        armRotator = hardwareMap.get(DcMotor.class, "ar");
        armExtender = hardwareMap.get(DcMotor.class, "ae");
        armLander = hardwareMap.get(DcMotor.class, "al");
        markerReleaser = hardwareMap.get(Servo.class, "mr");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        intakeMotor = hardwareMap.get(DcMotor.class, "im");

        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLander.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        armRotator.setDirection(DcMotor.Direction.FORWARD);
        armExtender.setDirection(DcMotor.Direction.FORWARD);
        armLander.setDirection(DcMotor.Direction.FORWARD);
        markerReleaser.setDirection(Servo.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        setupImu();

        TargetDirection.setImu(imu);

        getReady();
        
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        go();

    }

    //Sets up empty versions of methods to be called on init and start, respectively
    abstract public void getReady();

    abstract public void go();

    //Sets each individual drive's power based on forward, turn, and strafe  inputs
    protected void setDrive(double forwardPower, double turnPower, double strafePower) {
        forwardPower = Range.clip(forwardPower, -1.0, 1.0);
        turnPower = Range.clip(turnPower, -1.0, 1.0);
        strafePower = Range.clip(strafePower, -1.0, 1.0);
        backLeftDrive.setPower(forwardPower + turnPower - strafePower);
        backRightDrive.setPower(forwardPower - turnPower + strafePower);
        frontLeftDrive.setPower(forwardPower + turnPower + strafePower);
        frontRightDrive.setPower(forwardPower - turnPower - strafePower);
    }

    //Sets power of armRotator
    protected void setArmRotator(double turnPower) {
        isArmHolding = false;
        isMovingToGoal = false;
        armRotator.setPower(turnPower);
    }

    //Sets power of armExtender
    protected void setArmExtender(double extendPower) {
        armExtender.setPower(extendPower);
    }

    //Inits IMU and sets up its configuation and parameters
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

    //Sets power of armLander
    protected void setArmLander(double liftPower) {
        armLander.setPower(liftPower);
    }

    //Sets power of markerReleaser
    protected void setMarkerReleaser(double releasePosition) {
        markerReleaser.setPosition(releasePosition);
    }

    //Moves a robot forward or backwards based on time and power inputs
    protected void moveTime(double drivePower, long milliseconds, boolean isPositive, boolean isStrafing) {
        if (!isStrafing) {
            if (isPositive) {
                setDrive(drivePower, 0.0, 0.0);
                sleep(milliseconds);
                setDrive(0.0, 0.0, 0.0);
            } else {
                setDrive(-drivePower, 0.0, 0.0);
                sleep(milliseconds);
                setDrive(0.0, 0.0, 0.0);
            }
        } else {
            if (isPositive) {
                setDrive(0.0, 0.0, drivePower);
                sleep(milliseconds);
                setDrive(0.0, 0.0, 0.0);
            } else {
                setDrive(0.0, 0.0, -drivePower);
                sleep(milliseconds);
                setDrive(0.0, 0.0, 0.0);
            }
        }
    }

    //Calculates lander position
    protected int getArmLanderPosition() {
        return armLander.getCurrentPosition();
    }

    protected int getArmRotatorPosition() {
        return armRotator.getCurrentPosition();
    }

    protected void setIntakeMotor(double intakePower) {
        intakeMotor.setPower(intakePower);
    }


    protected void holdArmPosition() {
        if (!isArmHolding) {
            armHoldPID.setSetpoint(getArmRotatorPosition());
            armHoldPID.resetPID();
        }
        setArmRotator(armHoldPID.update(getArmRotatorPosition()));
        isArmHolding = true;
    }
}
