package org.firstinspires.ftc.teamcode;

public abstract class Autonomous_Parent extends Robot_Parent {

    // TODO: remember to set values to diff number
    private final double ENCODER_COUNTS_PER_INCH = 81.19;

    protected PID_Controller forwardPID = new PID_Controller(0.071,0.0,0.0);

    @Override
    public void initialize()
    {
        setup();
    }

    @Override
    public void play()
    {
        begin();
    }

    protected double getForwardPosition() {
        double position = backLeftDrive.getCurrentPosition() +
                frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition() +
                backRightDrive.getCurrentPosition();

        position /= 4.0;
        position /= ENCODER_COUNTS_PER_INCH;

        return position;
    }

    protected void driveDistance(double inches) {
        double goal = getForwardPosition() + inches;

        double multiplier = 1.0;
        if (inches < 0.0)
            multiplier = -1.0;

        setDrive(multiplier, 0.0);
        while (multiplier * (getForwardPosition() - goal) < 0.0) {
            setDrive(multiplier, 0.0);
        }
        setDrive(0.0, 0.0);
    }

    protected void turn(double degrees) {
        TargetDirection goalHeading = TargetDirection.makeTargetToRobotsRight(degrees);
        double multiplier = 1.0;
        if (degrees < 0.0)
            multiplier = -1.0;

        telemetry.addData("Degrees", degrees);
        telemetry.addData("Multiplier", multiplier);
        telemetry.addData("Current Relative Heading", goalHeading.calculateDistanceFromTarget());
        telemetry.update();
        sleep(3000);

        setDrive(0.0, multiplier);
        while (multiplier * goalHeading.calculateDistanceFromTarget() < 0.0) {
            setDrive(0.0, multiplier);
        }
        setDrive(0.0, 0.0);
    }
}