package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

public abstract class Autonomous_Parent extends Robot_Parent {

    protected PID_Controller forwardPID = new PID_Controller(0.071, 0.0, 0.0);
    protected PID_Controller strafePID = new PID_Controller(0.071,0.0,0.0);
    protected PID_Controller turnPID = new PID_Controller(0.025, 0.0, 0.0);

    private final double EC_PER_FEET = 974.28;
    private final boolean USE_LEFT_ENCODER = true;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    BNO055IMU imu;




    @Override
    public void initializeRobot() {
        setupIMU();
        startUp();
    }

    @Override
    public void startRobot(){

    }
    public void runAutonomous() {

    }

    protected void moveForwardEncoder(double distanceInFeet) {// move forward encoder based which allows you to drive using distance based.
        if (USE_LEFT_ENCODER)
            moveForwardLeftEncoder(distanceInFeet, ENCODER_DRIVE_POWER);//
        else
            moveForwardRightEncoder(distanceInFeet, ENCODER_DRIVE_POWER);//
    }

    protected void moveBackwardEncoder(double distanceInFeet) {// move backward  encoder based which allows you to drive using distance based.
        if (USE_LEFT_ENCODER)
            moveBackwardLeftEncoder(distanceInFeet, ENCODER_DRIVE_POWER);
        else
            moveBackwardRightEncoder(distanceInFeet, ENCODER_DRIVE_POWER);
    }

    protected void moveForwardEncoder(double distanceInFeet, double drivePower) {
        if (USE_LEFT_ENCODER)
            moveForwardLeftEncoder(distanceInFeet, drivePower);
        else
            moveForwardRightEncoder(distanceInFeet, drivePower);
    }

    protected void moveBackwardEncoder(double distanceInFeet, double drivePower) {
        if (USE_LEFT_ENCODER)
            moveBackwardLeftEncoder(distanceInFeet, drivePower);
        else
            moveBackwardRightEncoder(distanceInFeet, drivePower);
    }

    private void moveForwardRightEncoder(double distanceInFeet, double drivePower) {
        int targetPos = backRightDrive.getCurrentPosition() + (int) (distanceInFeet * EC_PER_FEET);
        setDrive(drivePower, drivePower);
        while (backRightDrive.getCurrentPosition() < targetPos && opModeIsActive())
        {
            telemetry.addLine("Test: While Current Position < Goal");
            telemetry.addData("Current Position","%d",backRightDrive.getCurrentPosition());
            telemetry.addData("Goal","%d",targetPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void moveBackwardRightEncoder(double distanceInFeet, double drivePower) {
        int targetPos = backRightDrive.getCurrentPosition() - (int) (distanceInFeet * EC_PER_FEET);
        setDrive(-drivePower, -drivePower);
        while (backRightDrive.getCurrentPosition() > targetPos && opModeIsActive())
        {
            telemetry.addLine("Test: While Current Position < Goal");
            telemetry.addData("Current Position","%d",backRightDrive.getCurrentPosition());
            telemetry.addData("Goal","%d",targetPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void moveForwardLeftEncoder(double distanceInFeet, double drivePower) {
        int targetPos = backLeftDrive.getCurrentPosition() + (int) (distanceInFeet * EC_PER_FEET);
        setDrive(drivePower, drivePower);
        while (backLeftDrive.getCurrentPosition() < targetPos && opModeIsActive())
        {
            telemetry.addLine("Test: While Current Position < Goal");
            telemetry.addData("Current Position","%d",backLeftDrive.getCurrentPosition());
            telemetry.addData("Goal","%d",targetPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void moveBackwardLeftEncoder(double distanceInFeet, double drivePower) {
        int targetPos = backLeftDrive.getCurrentPosition() - (int) (distanceInFeet * EC_PER_FEET);
        setDrive(-drivePower, -drivePower);
        while (backLeftDrive.getCurrentPosition() > targetPos && opModeIsActive())
        {
            telemetry.addLine("Test: While Current Position < Goal");
            telemetry.addData("Current Position","%d",backLeftDrive.getCurrentPosition());
            telemetry.addData("Goal","%d",targetPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }


    protected double getForwardPosition() {
        double position = backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition() +
                frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition();
        position /= 4.0;
        position /= EC_PER_FEET / 12;
        return position;
    }

    protected double getStrafePosition(){
        double position = -backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition()
                + frontLeftDrive.getCurrentPosition() - frontRightDrive.getCurrentPosition();
        position /= 4.0;
        position /= EC_PER_FEET / 12;
        return position;
    }
    protected void startUp() {

    }
    private void setupIMU()
    {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }
}
