package org.firstinspires.ftc.parent_classes;

import org.firstinspires.ftc.teamcode.PID_Controller;
import org.firstinspires.ftc.teamcode.TargetDirection;

public abstract class Autonomous_Parent extends Robot_Parent {

    // TODO: remember to set values to diff number
    private final double ENCODER_COUNTS_PER_INCH = 81.19;
    private final double EARLY_STOP_DEGREES = 5.0;
    private final double MILLIS_PER_INCH = 27.7;

    protected Sampler sampler = new Sampler();
    protected Sampler.GoldPosition position = Sampler.GoldPosition.UNKNOWN;

    protected PID_Controller forwardPID = new PID_Controller(0.071, 0.0, 0.0);

    @Override
    public void initialize() {
        setup();
        sampler.init(telemetry, hardwareMap);
    }

    @Override
    public void play() {
        position = sampler.sample();
        begin();
    }

    protected double getForwardPosition() {
        double position;
        position = backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition() + frontLeftDrive.getCurrentPosition() + frontLeftDrive.getCurrentPosition();
        position /= 4.0;

        position /= ENCODER_COUNTS_PER_INCH;

        return position;
    }

    protected int getLandingMotorPosition() {
        return landingMotor.getCurrentPosition();
    }

    protected void land() {
        // TODO: Change values to right number
        int encoderCountsToLand = 0;
        double driveDistanceAfterLanding = 0.0;

        int startLiftPosition = getLandingMotorPosition();
        // lower robot
        int goal = startLiftPosition + encoderCountsToLand;
        setLandingMotorPower(1.0);
        while (getLandingMotorPosition() < goal && opModeIsActive()) ;
        setLandingMotorPower(0.0);

        driveDistanceEncoder(driveDistanceAfterLanding);

        goal = startLiftPosition;
        setLandingMotorPower(-1.0);
        while (getLandingMotorPosition() > goal && opModeIsActive()) ;
        setLandingMotorPower(0.0);
    }

    protected void sampleDepot() {
        switch (position) {
            case LEFT:
                turnDegreesPID(-27.5, 2000);
                setIntake(1.0);
                driveDistanceTime(44.0);
                setIntake(0.0);
                turnDegreesPID(72.5, 4000);
                driveDistanceTime(30.0);
                turnPID(TargetDirection.makeTargetAtFieldPosition(180.0), 3000);
                break;
            case RIGHT:
                turnDegreesPID(27.5, 2000);
                setIntake(1.0);
                driveDistanceTime(44.0);
                setIntake(0.0);
                turnDegreesPID(-72.5, 4000);
                driveDistanceTime(30.0);
                turnPID(TargetDirection.makeTargetAtFieldPosition(180.0), 5000);
                break;
            default:
                setIntake(1.0);
                driveDistanceTime(60.0);
                setIntake(0.0);
                turnPID(TargetDirection.makeTargetAtFieldPosition(180.0), 4000);
                break;
        }
    }

    protected void sampleCrater() {
        // TODO: Change values
        switch (position) {
            case LEFT:
                turnDegreesPID(-27.5, 2000);
                setIntake(1.0);
                driveDistanceTime(33.5);
                setIntake(0.0);
                driveDistanceTime(-34.0);
                turnDegreesPID(27.5, 2000);
                break;
            case RIGHT:
                turnDegreesPID(27.5, 2000);
                setIntake(1.0);
                driveDistanceTime(33.5);
                setIntake(0.0);
                driveDistanceTime(34.0);
                turnDegreesPID(-27.5, 2000);
                break;
            default:
                setIntake(1.0);
                driveDistanceTime(33.5);
                setIntake(0.0);
                driveDistanceTime(-34.0);
                break;
        }
    }

    protected void parkDepot() {
        driveDistanceTime(-80.0);

    }

    protected void parkCrater() {

    }

    protected void placeTeamMarker() {
        //TODO: Placing Marker (Mechanism needed)

    }

    protected void driveToDepot() {
        // 170, 44, 90, 55, -78
        turnPID(TargetDirection.makeTargetAtFieldPosition(170.0), 2000);
        driveDistanceTime(44.0);
        turnPID(TargetDirection.makeTargetAtFieldPosition(90.0), 5000);
        driveDistanceTime(55.0);
    }

    protected void driveDistanceTime(double inches) {
        long endTime = System.currentTimeMillis() + Math.round(MILLIS_PER_INCH * Math.abs(inches));

        if (inches > 0.0)
            setArcadeDrive(1.0, 0.0);
        else
            setArcadeDrive(-1.0, 0.0);

        while (System.currentTimeMillis() < endTime && opModeIsActive()) ;

        setArcadeDrive(0.0, 0.0);
    }

    protected void driveDistanceEncoder(double inches) {
        double goal = getForwardPosition() + inches;
        double multiplier = 1.0;

        if (inches < 0.0)
            multiplier = -1.0;

        setArcadeDrive(multiplier, 0.0);
        while (multiplier * (getForwardPosition() - goal) < 0.0 && opModeIsActive()) {
            setArcadeDrive(multiplier, 0.0);
        }
        setArcadeDrive(0.0, 0.0);
    }

    protected void turnDegreesIMU(double degrees) {
        if (degrees > EARLY_STOP_DEGREES) {
            degrees = Math.max(EARLY_STOP_DEGREES, degrees - EARLY_STOP_DEGREES);
        } else if (degrees < -EARLY_STOP_DEGREES) {
            degrees = Math.min(-EARLY_STOP_DEGREES, degrees + EARLY_STOP_DEGREES);
        }

        TargetDirection goalHeading = TargetDirection.makeTargetToRobotsRight(degrees);
        double multiplier = 1.0;
        if (degrees < 0.0)
            multiplier = -1.0;

        setArcadeDrive(0.0, multiplier);
        while (multiplier * goalHeading.calculateDistanceFromTarget() < 0.0 && opModeIsActive()) {
            setArcadeDrive(0.0, multiplier);
        }
        setArcadeDrive(0.0, 0.0);
    }

    protected void turnPID(TargetDirection target, long millis) {
        long endTime = System.currentTimeMillis() + millis;
        goToTurnPID.setSetpoint(0.0);
        goToTurnPID.resetPID();
        while (System.currentTimeMillis() < endTime && opModeIsActive()) {
            double turnPower = goToTurnPID.update(target.calculateDistanceFromTarget());
            setArcadeDrive(0.0, turnPower);
        }
    }

    protected void turnDegreesPID(double degrees, long millis) {
        TargetDirection target = TargetDirection.makeTargetToRobotsRight(degrees);
        turnPID(target, millis);
    }
}