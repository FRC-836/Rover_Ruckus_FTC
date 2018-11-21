package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.BensCV.Sampler;

public abstract class Autonomous_Parent extends Robot_Parent {

    private ElapsedTime runtime = new ElapsedTime();
    protected PID_Controller forwardPID = new PID_Controller(0.071, 0.0, 0.0);
    protected PID_Controller strafePID = new PID_Controller(0.071,0.0,0.0);
    protected PID_Controller turnPID = new PID_Controller(0.025, 0.0, 0.0);

    private Sampler sampler;


    private final double EC_PER_IN_DRIVE = 104.7;
    private final double SECONDS_PER_IN = 0.16;
    private final double SECONDS_PER_DEGREE = 0.03;
    private final double SAMPLE_TURN_ANGLE = 26.36;

    protected Sampler.position goldTarget = Sampler.position.NONE;

    @Override
    public void getReady() {
        sampler = new Sampler(false,false, hardwareMap, telemetry, false);
        sampler.initialize();

    }

    @Override
    public void go() {
        goldTarget = sampler.run();
        while(opModeIsActive())
        telemetry.clear();
        telemetry.addData("Total runtime", "%6.3f seconds", runtime.seconds());
        //This lets us know how long our autonomous lasts for
        telemetry.update();
    }

    protected double getForwardPosition() {
        double position = backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition() +
                frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition();
        position /= 4.0;
        position /= EC_PER_IN_DRIVE;
        return position;
    }

    protected double getStrafePosition(){
        double position = -backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition()
                + frontLeftDrive.getCurrentPosition() - frontRightDrive.getCurrentPosition();
        position /= 4.0;
        position /= EC_PER_IN_DRIVE;
        return position;
    }

    protected double getArmPosition() {
        double position = armRotator.getCurrentPosition();
        position /= EC_PER_DEGREE_ARM;
        return position;
    }

    protected double getExtenderPosition() {
        double position = armExtender.getCurrentPosition();
        position /= EC_PER_IN_ARM;
        return position;
    }

    // Drive functions involving PID Controllers

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
    protected void turnToFieldPID(double degrees){
        TargetDirection goal = TargetDirection.makeTargetAtFieldPosition(degrees);
        turnToTargetPID(goal);
    }

    protected void turnRightPID(double degrees) {
        TargetDirection goal = TargetDirection.makeTargetToRobotsRight(degrees);
        turnToTargetPID(goal);
    }

    protected void turnToTargetPID(TargetDirection goal) {
        turnPID.setSetpoint(0.0);
        turnPID.resetPID();
        long endTime = System.currentTimeMillis() + (long) (SECONDS_PER_DEGREE * 1000.0 * Math.abs(goal.calculateDistanceFromTarget()));
        while (opModeIsActive() && (System.currentTimeMillis() <= endTime)) {
            setDrive(0.0, turnPID.update(goal.calculateDistanceFromTarget()), 0.0);
        }
        setDrive(0.0, 0.0, 0.0);
    }

    // Simple drive functions

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

    @Deprecated
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

    protected void armRotateToPosition(double degrees) {
    }

    // Task-based Functions
    protected void deploy() {
        setArmRotator(0.01);
        long startTime = System.currentTimeMillis();
        while(opModeIsActive() && startTime + 500 > System.currentTimeMillis());
        setArmExtender(0.01);
        setArmRotator(0.0);
        setArmExtender(0.0);
    }

    protected void sampleDepotSide() {
        double driveToGoldDrive = 48.0;
        double setGrid = 0.0;
        double driveToDepot = 31.0;
        switch (goldTarget)
        {
            case LEFT:
                turnRightPID(-SAMPLE_TURN_ANGLE);
                driveDistancePID(driveToGoldDrive);
                turnToFieldPID(setGrid);
                driveDistancePID(-driveToDepot);
                break;
            case RIGHT:
                turnRightPID(SAMPLE_TURN_ANGLE);
                driveDistancePID(driveToGoldDrive);
                turnToFieldPID(setGrid);
                driveStrafePID(driveToDepot);
                break;
            case NONE:
            case CENTER:
            default:
                driveDistancePID(65.0);
                turnToFieldPID(setGrid);
                break;
        }
    }

    protected void sampleCraterSide() {
        double longDrive = 29.0;
        double shortDrive = 25.25;

        switch (goldTarget)
        {
            case LEFT:
                turnRightPID(-SAMPLE_TURN_ANGLE);
                driveDistancePID(longDrive);
                driveDistancePID(-longDrive);
                break;
            case RIGHT:
                turnRightPID(SAMPLE_TURN_ANGLE);
                driveDistancePID(longDrive);
                driveDistancePID(-longDrive);
                break;
            case NONE:
            case CENTER:
            default:
                driveDistancePID(shortDrive);
                driveDistancePID(-shortDrive);
                break;
        }
        turnToFieldPID(176.0);
    }
    protected void goToDepotCraterSide() {
        driveDistancePID(42.0);
        turnToFieldPID(0.0);
        driveStrafePID(68.5);
    }

    protected void releaseMarker() {
        //Release Code
    }

    protected void parkInCraterCraterSide() {
        driveStrafePID(-84.0);
    }
    protected void parkInCraterDepotSide() {
        driveDistancePID(84.0);
    }

}
