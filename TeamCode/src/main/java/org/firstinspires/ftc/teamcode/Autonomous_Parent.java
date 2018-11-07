package org.firstinspires.ftc.teamcode;

import static com.sun.tools.doclint.Entity.and;
import static com.sun.tools.doclint.Entity.mu;
import static com.sun.tools.doclint.Entity.or;
import static com.sun.tools.doclint.Entity.thorn;

public abstract class Autonomous_Parent extends Robot_Parent {

    protected PID_Controller forwardPID = new PID_Controller(0.071, 0.0, 0.0);
    protected PID_Controller strafePID = new PID_Controller(0.071, 0.0, 0.0);
    protected PID_Controller turnPID = new PID_Controller(0.025, 0.0, 0.0);

    private final double EC_PER_IN = 104.7;
    private final double SECONDS_PER_IN = 0.16;
    private final double SECONDS_PER_DEGREE = 0.03;
    private final long CYCLE_TIME_MS = 5;

    @Override
    public void getReady() {

    }

    @Override
    public void go() {

    }

    protected double getForwardPosition() {
        double position = backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition() +
                frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition();
        position /= 4.0;
        position /= EC_PER_IN;
        return position;
    }

    protected double getStrafePosition() {
        double position = -backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition()
                + frontLeftDrive.getCurrentPosition() - frontRightDrive.getCurrentPosition();
        position /= 4.0;
        position /= EC_PER_IN;
        return position;
    }

    protected void driveDistancePID(double inches) {
        Heading retain = Heading.createRelativeHeading(0.0f);
        forwardPID.setSetpoint(getForwardPosition() + inches);
        holdTurnPID.setSetpoint(0.0);
        forwardPID.resetPID();
        holdTurnPID.resetPID();
        int iterations = 0;
        int numIterations = (int) (SECONDS_PER_IN * 1000.0 * inches / CYCLE_TIME_MS);
        while (opModeIsActive() && (iterations <= numIterations)) {
            setDrive(forwardPID.update(getForwardPosition()), holdTurnPID.update(retain.getRelativeHeading()), 0.0);
            iterations++;
            sleep(CYCLE_TIME_MS);
        }
        setDrive(0.0, 0.0, 0.0);
    }

    protected void driveStrafePID(double inches) {
        Heading retain = Heading.createRelativeHeading(0.0f);
        strafePID.setSetpoint(getStrafePosition() + inches);
        holdTurnPID.setSetpoint(0.0);
        strafePID.resetPID();
        holdTurnPID.resetPID();
        int iterations = 0;
        int numIterations = (int) (SECONDS_PER_IN * 1000.0 * inches / CYCLE_TIME_MS);
        while (opModeIsActive() && (iterations <= numIterations)) {
            setDrive(0.0, holdTurnPID.update(retain.getRelativeHeading()), strafePID.update(getStrafePosition()));
            iterations++;
            sleep(CYCLE_TIME_MS);
        }
        setDrive(0.0, 0.0, 0.0);
    }

    protected void driveTurnPID(double degrees) {
        Heading goal = Heading.createRelativeHeading((float) degrees);
        turnPID.setSetpoint(0.0);
        turnPID.resetPID();
        int iterations = 0;
        int numIterations = (int) (SECONDS_PER_DEGREE * 1000.0 * degrees / CYCLE_TIME_MS);
        while (opModeIsActive() && (iterations <= numIterations)) {
            setDrive(0.0, turnPID.update(goal.getRelativeOffset()), 0.0);
            iterations++;
            sleep(CYCLE_TIME_MS);
        }
        setDrive(0.0, 0.0, 0.0);
    }

    protected void driveDistance(double inches) {
        Heading retain = Heading.createRelativeHeading(0.0f);
        holdTurnPID.setSetpoint(0.0);
        holdTurnPID.resetPID();
        double finalPosition = getForwardPosition() + inches;
        double multiplier = 1.0;
        if (inches < 0.0)
            multiplier = -1.0;
        while (getForwardPosition() * multiplier < finalPosition * multiplier && opModeIsActive()) {
            setDrive(multiplier, holdTurnPID.update(retain.getRelativeOffset()), 0.0);
        }
        setDrive(0.0, 0.0, 0.0);
    }

    protected void driveStrafe(double inches) {
        Heading retain = Heading.createRelativeHeading(0.0f);
        holdTurnPID.setSetpoint(0.0);
        holdTurnPID.resetPID();
        double finalPosition = getStrafePosition() + inches;
        double multiplier = 1.0;
        if (inches < 0.0)
            multiplier = -1.0;
        while (getStrafePosition() * multiplier < finalPosition * multiplier && opModeIsActive()) {
            setDrive(0.0, holdTurnPID.update(retain.getRelativeOffset()), multiplier);
        }
        setDrive(0.0, 0.0, 0.0);
    }

    protected void driveTurn(double degrees) {
        Heading goal = Heading.createRelativeHeading((float)-degrees);
        double multiplier = 1.0;
        if (degrees < 0.0)
            multiplier = -1.0;
        while (goal.getRelativeHeading() * multiplier < 0.0 && opModeIsActive()) {
            setDrive(0.0, multiplier, 0.0);
        }
        setDrive(0.0, 0.0, 0.0);
    }
}