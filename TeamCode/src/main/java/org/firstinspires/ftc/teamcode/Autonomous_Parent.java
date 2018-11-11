package org.firstinspires.ftc.teamcode;

public abstract class Autonomous_Parent extends Robot_Parent {

    protected PID_Controller forwardPID = new PID_Controller(0.071, 0.0, 0.0);
    protected PID_Controller strafePID = new PID_Controller(0.071,0.0,0.0);
    protected PID_Controller turnPID = new PID_Controller(0.025, 0.0, 0.0);
    
    private final double EC_PER_IN = 104.7;
    private final double SECONDS_PER_IN = 0.16;
    private final double SECONDS_PER_DEGREE = 0.03;

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

    protected double getStrafePosition(){
        double position = -backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition()
                + frontLeftDrive.getCurrentPosition() - frontRightDrive.getCurrentPosition();
        position /= 4.0;
        position /= EC_PER_IN;
        return position;
    }

    protected void driveDistancePID(double inches) {
        TargetDirection retain = TargetDirection.makeTargetToRobotsRight(0.0);
        forwardPID.setSetpoint(getForwardPosition() + inches);
        holdTurnPID.setSetpoint(0.0);
        forwardPID.resetPID();
        holdTurnPID.resetPID();
        long endTime = System.currentTimeMillis() + (long) (SECONDS_PER_IN * 1000.0 * Math.abs(inches));
        while (opModeIsActive() && (System.currentTimeMillis() <= endTime)) {
            setDrive(forwardPID.update(getForwardPosition()), holdTurnPID.update(retain.calculateDistanceFromTarget()), 0.0);
        }
        setDrive(0.0, 0.0, 0.0);
    }

    protected void driveStrafePID(double inches) {
        TargetDirection retain = TargetDirection.makeTargetToRobotsRight(0.0);
        strafePID.setSetpoint(getStrafePosition() + inches);
        holdTurnPID.setSetpoint(0.0);
        strafePID.resetPID();
        holdTurnPID.resetPID();
        long endTime = System.currentTimeMillis() + (long) (SECONDS_PER_IN * 1000.0 * Math.abs(inches));
        while (opModeIsActive() && (System.currentTimeMillis() <= endTime)) {
            setDrive(0.0, holdTurnPID.update(retain.calculateDistanceFromTarget()), strafePID.update(getStrafePosition()));
        }
        setDrive(0.0, 0.0, 0.0);
    }

    protected void driveTurnPID(double degrees) {
        TargetDirection goal = TargetDirection.makeTargetToRobotsRight(degrees);
        turnPID.setSetpoint(0.0);
        turnPID.resetPID();
        long endTime = System.currentTimeMillis() + (long) (SECONDS_PER_DEGREE * 1000.0 * Math.abs(degrees));
        while (opModeIsActive() && (System.currentTimeMillis() <= endTime)) {
            setDrive(0.0, turnPID.update(goal.calculateDistanceFromTarget()), 0.0);
        }
        setDrive(0.0, 0.0, 0.0);
    }

    protected void driveDistance(double inches) {
        TargetDirection retain = TargetDirection.makeTargetToRobotsRight(0.0);
        holdTurnPID.setSetpoint(0.0);
        holdTurnPID.resetPID();
        double finalPosition = getForwardPosition() + inches;
        double multiplier = 1.0;
        if (inches < 0.0)
            multiplier = -1.0;
        while (getForwardPosition() * multiplier < finalPosition * multiplier && opModeIsActive()) {
            setDrive(multiplier, holdTurnPID.update(retain.calculateDistanceFromTarget()), 0.0);
        }
        setDrive(0.0, 0.0, 0.0);
    }

    protected void driveStrafe(double inches) {
        TargetDirection retain = TargetDirection.makeTargetToRobotsRight(0.0);;
        holdTurnPID.setSetpoint(0.0);
        holdTurnPID.resetPID();
        double finalPosition = getStrafePosition() + inches;
        double multiplier = 1.0;
        if (inches < 0.0)
            multiplier = -1.0;
        while (getStrafePosition() * multiplier < finalPosition * multiplier && opModeIsActive()) {
            setDrive(0.0, holdTurnPID.update(retain.calculateDistanceFromTarget()), multiplier);
        }
        setDrive(0.0, 0.0, 0.0);
    }

    protected void driveTurn(double degrees) {
        TargetDirection goal = TargetDirection.makeTargetToRobotsRight(degrees);
        double multiplier = 1.0;
        if (degrees < 0.0)
            multiplier = -1.0;
        while (goal.calculateDistanceFromTarget() * multiplier < 0.0 && opModeIsActive()) {
            setDrive(0.0, multiplier, 0.0);
        }
        setDrive(0.0, 0.0, 0.0);
    }


}
