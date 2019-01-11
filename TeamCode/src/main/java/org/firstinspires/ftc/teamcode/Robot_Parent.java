package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public abstract class Robot_Parent extends LinearOpMode {
    //Set up robot parts/controllers
    protected DcMotor backLeftDrive;
    protected DcMotor backRightDrive;
    protected DcMotor frontLeftDrive;
    protected DcMotor frontRightDrive;
    protected DcMotor armRotator;
    protected DcMotor armExtender;
    protected DcMotor armLander;
    protected Servo markerReleaser;
    private BNO055IMU imu;
    private BNO055IMU armImu;
    protected DcMotor intakeMotor;
    protected boolean isMovingToGoal = false;

    protected boolean armHasBeenHolding = false;

    protected final double ARM_POSITION_UP = 70.0;
    protected final double ARM_POSITION_DOWN = 170.0;
    private final double ARM_ROTATOR_ENCODER_TO_ANGLE = 1.0 / 9.5; // 10 encoder counts per degree
    private double armRotatorDrift;

    public double pStableHoldTurn = 0.019;
    public double dStableHoldTurn = 0.00195;

    protected PID_Controller holdTurnPID = new PID_Controller(pStableHoldTurn, 0.0, dStableHoldTurn);

    protected PID_Controller armHoldP = new PID_Controller(0.0076, 0.0, 0.0);
    protected PID_Controller armHoldD = new PID_Controller(0.0, 0.0, 0.0015);
    boolean useP = true;

    private final double K_GRAVITY = 0.3;
    protected boolean isAuto;
    protected double armRotatorPower = 0.0;

    //Maps robot parts to data values in config file, sets up opMode
    @Override
    public void runOpMode() {
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
        armImu = hardwareMap.get(BNO055IMU.class, "arm_imu");

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
        ArmTargetDirection.setImu(armImu);

        getReady();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        // Calibrate arm encoder
        double armPos = ArmTargetDirection.getPitch();
        armImu.close();
        if (armPos < 5.0) // Invalid
            armPos = 50.0;
        calibrateArmRotatorEncoder(getArmRotatorEstimate_Drifting(), armPos);

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

    protected void setArmRotator(double armPower) {
        setArmRotator(armPower, !isAuto);
    }

    protected void setArmRotator(double armPower, boolean useGravity) {
        armHasBeenHolding = false;
        isMovingToGoal = false;

        armRotatorPower = armPower;

        if (useGravity)
            armPower += K_GRAVITY * Math.cos(Math.toRadians(getArmRotatorPosition()));

        armRotator.setPower(armPower);
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
        armImu.initialize(imuParameters);
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

    private double getArmRotatorEstimate_Drifting()
    {
        double armPosition = armRotator.getCurrentPosition();
        armPosition *= ARM_ROTATOR_ENCODER_TO_ANGLE;
        return armPosition;
    }

    private double getArmRotatorEstimate_Calibrated()
    {
        double armPosition = getArmRotatorEstimate_Drifting();
        armPosition += armRotatorDrift;
        return armPosition;
    }

    private void calibrateArmRotatorEncoder(double driftingArmReading, double imuArmReading)
    {
        armRotatorDrift = imuArmReading - driftingArmReading;
    }

    protected double getArmRotatorPosition() {
        /*double armPosition = ArmTargetDirection.getPitch();

        if (armPosition < 5.0)
        { // Invalid IMU Reading
            return getArmRotatorEstimate_Calibrated();
        }
        else
        { // Valid IMU Reading
            calibrateArmRotatorEncoder(getArmRotatorEstimate_Drifting(), armPosition);
            return armPosition;
        }
        */
        return getArmRotatorEstimate_Calibrated();
    }

    protected void setIntakeMotor(double intakePower) {
        intakeMotor.setPower(intakePower);
    }

    private double transformArmPosition(double armRotatorPosition) {
        double m = -238.0 / (-ARM_POSITION_UP + ARM_POSITION_DOWN + 318.0);
        double b = -m * ARM_POSITION_UP;
        return armRotatorPosition;//(int) (((m * armRotatorPosition) + b) + armRotatorPosition);
    }

    protected void holdArmPosition() {
        holdArmPosition(transformArmPosition(getArmRotatorPosition()));
    }

    protected void holdArmPosition(double armPositionToHold) {
        if (!armHasBeenHolding) {
            double position = transformArmPosition(armPositionToHold);
            armHoldP.setSetpoint(position);
            armHoldP.resetPID();
            armHoldD.setSetpoint(position);
            armHoldD.resetPID();
        }
        // Always use the derivative controller
        double power = armHoldD.update(getArmRotatorPosition());

        if (useP)
            // If using the proportional controller, add the proportional component
            power += armHoldP.update(getArmRotatorPosition());
        else
            // If NOT using the proportional controller, at least keep the controller updated
            armHoldP.update(getArmRotatorPosition());
        setArmRotator(power);
        armHasBeenHolding = true;
    }
}