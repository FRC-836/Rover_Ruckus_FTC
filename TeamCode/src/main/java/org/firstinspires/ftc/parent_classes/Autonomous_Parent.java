package org.firstinspires.ftc.parent_classes;

import org.firstinspires.ftc.teamcode.PID_Controller;
import org.firstinspires.ftc.teamcode.TargetDirection;

public abstract class Autonomous_Parent extends Robot_Parent {

    // TODO: remember to set values to diff number
    private final double ENCODER_COUNTS_PER_INCH = 81.19;
    private final double EARLY_STOP_DEGREES = 5.0;

    private Sampler sampler = new Sampler();
    protected Sampler.GoldPosition position;

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

    protected int getLandingMotorPosition() {return landingMotor.getCurrentPosition();}

    protected void land() {
        // TODO: Change values to right number
        int encoderValue = 0;

        // lower robot
        int goal = getLandingMotorPosition() + encoderValue;
        setLandingMotorPower(1.0);
        while ((getLandingMotorPosition() - goal) < 0.0 && opModeIsActive()) {
        }
        setLandingMotorPower(0.0);


    }

    protected void sample() {
        /*TODO: Insert Sampling Code:
        Must return a case weather left, right, or center; must complete a condition for each.
        Also, this is JUST for if facing crater.
         */
    }

    protected void collectMineral() {
        //TODO: Insert Intake Code (Also have the robot move forward for after landing/starting)
    }

    protected void placeTeamMarker() {
        //TODO: Placing Marker (Mechanism needed)
    }

    protected void scoreMineralInDepot() {
        //TODO: Insert Scoring code in depot (Mechanism needed)
    }

    protected void driveToDepot(boolean sample, boolean collect) {
        if (sample) {
            driveDistance(1.5);
            sample();
            driveDistance(1.0);
        } else if (collect) {
            driveDistance(1.5);
            collectMineral();
            driveDistance(1.0);
        } else {
            driveDistance(2.5);
        }
    }

    protected void driveFromCraterToDepot(boolean driveDirectlyRight) {
        land();
        driveDistance(1.0);
        sample();
        if (driveDirectlyRight) {
            turnDegrees(-70.0);
            driveDistance(1.0);
            turnDegrees(-35.0);
            driveDistance(2.8);
        } else {
            turnDegrees(-175.0);
            driveDistance(1.3);
            turnDegrees(40.0);
            driveDistance(1.6);
            turnDegrees(-90.0);
            driveDistance(1.4);
        }
    }

    protected void driveFromDepotToPark(boolean driveDirectlyLeft) {
        land();
        //TODO Change ALL values; These are estimates!
        if (driveDirectlyLeft) {
            turnDegrees(35.0);
            driveDistance(3.0);
        } else {
            turnDegrees(180.0);
            driveDistance(2.5);
            turnDegrees(-45.0);
            driveDistance(1.2);
            turnDegrees(-90.0);
            driveDistance(1.7);
            turnDegrees(90.0);
            driveDistance(0.5);
        }
    }

    protected void driveDistance(double inches) {
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

    protected void turnDegrees(double degrees) {
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
}