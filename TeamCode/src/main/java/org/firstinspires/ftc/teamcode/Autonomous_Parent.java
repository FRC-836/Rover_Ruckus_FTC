package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.BensCV.Sampler;

public abstract class Autonomous_Parent extends Robot_Parent {
    //Set up variables/controllers
    private ElapsedTime runtime = new ElapsedTime();
    protected PID_Controller forwardPID = new PID_Controller(0.071, 0.0, 0.02);
    protected PID_Controller strafePID = new PID_Controller(0.071,0.0,0.02);
    protected PID_Controller turnPID = new PID_Controller(0.025, 0.0, 0.0);

    private Sampler sampler;

    private final double EC_PER_IN_DRIVE = 91.37;
    private final double SECONDS_PER_IN = 0.16;
    private final double SECONDS_PER_DEGREE = 0.03;
    private final double SAMPLE_TURN_ANGLE = 26.36;

    protected Sampler.position goldTarget = Sampler.position.NONE;

    //Sets up CV on init
    @Override
    public void getReady() {
        sampler = new Sampler(false, hardwareMap, telemetry, false);
        sampler.initialize();
    }

    //Sets up CV and time measurement, activates on start
    @Override
    public void go() {
        goldTarget = sampler.run();
        telemetry.clear();
        telemetry.addData("Total runtime", "%6.3f seconds", runtime.seconds());
        //This lets us know how long our autonomous lasts for
        telemetry.update();
    }

    // Calculates forward position by averaging the estimates from all four motors.
    protected double getForwardPosition() {
        double position = backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition() +
                frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition();
        position /= 4.0;
        position /= EC_PER_IN_DRIVE;
        return position;
    }

    // Calculates strafe position by averaging the estimates from all four motors.
    protected double getStrafePosition(){
        double position = -backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition()
                + frontLeftDrive.getCurrentPosition() - frontRightDrive.getCurrentPosition();
        position /= 4.0;
        position /= EC_PER_IN_DRIVE;
        return position;
    }
    //Calculates arm position
    protected double getArmRotatorPosition() {
        double position = armRotator.getCurrentPosition();
        position /= EC_PER_DEGREE_ARM;
        return position;
    }
    //Calculates extender position
    protected double getArmExtenderPosition() {
        double position = armExtender.getCurrentPosition();
        position /= EC_PER_IN_ARM;
        return position;
    }

    // Drive functions involving PID Controllers

    /*
    Drives forward in inches using forwardPID
    Uses holdTurnPID to attempt to drive straight.
    Drives for a predicted amount of time based on SECONDS_PER_IN
     */
    protected void driveDistancePID(double inches, long endTimeMillis) {
        TargetDirection retain = TargetDirection.makeTargetToRobotsRight(0.0);
        forwardPID.setSetpoint(getForwardPosition() + inches);
        strafePID.setSetpoint(getStrafePosition());
        holdTurnPID.setSetpoint(0.0);
        forwardPID.resetPID();
        strafePID.resetPID();
        holdTurnPID.resetPID();
        endTimeMillis += System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() <= endTimeMillis)) {
            setDrive(forwardPID.update(getForwardPosition()), holdTurnPID.update(retain.calculateDistanceFromTarget()), strafePID.update(getStrafePosition()));
        }
        setDrive(0.0, 0.0, 0.0);
    }

    /*Drives forward in inches using Strafe PID
     Uses holdTurnPID to attempt to drive straight
     Drives for a predicted amount of time based on SECONDS_PER_IN
     */
    protected void driveStrafePID(double inches, long endTimeMillis) {
        TargetDirection retain = TargetDirection.makeTargetToRobotsRight(0.0);
        forwardPID.setSetpoint(getForwardPosition());
        strafePID.setSetpoint(getStrafePosition() + inches);
        holdTurnPID.setSetpoint(0.0);
        forwardPID.resetPID();
        strafePID.resetPID();
        holdTurnPID.resetPID();
        endTimeMillis += System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() <= endTimeMillis)) {
            setDrive(forwardPID.update(getForwardPosition()), holdTurnPID.update(retain.calculateDistanceFromTarget()), strafePID.update(getStrafePosition()));
        }
        setDrive(0.0, 0.0, 0.0);
    }
    protected void turnToFieldPID(double degrees, long timeInMillis){
        TargetDirection goal = TargetDirection.makeTargetAtFieldPosition(degrees);
        turnToTargetPID(goal, timeInMillis);
    }
    //Uses TargetDirections to turn to the right using turnPID
    protected void turnRightPID(double degrees, long timeInMillis) {
        TargetDirection goal = TargetDirection.makeTargetToRobotsRight(degrees);
        turnToTargetPID(goal, timeInMillis);
    }
     /*Uses turn PID to turn to a specified point and turns for a predicted amount of time with
       SECONDS_PER_DEGREE
     */
    protected void turnToTargetPID(TargetDirection goal, long timeInMillis) {
        turnPID.setSetpoint(0.0);
        turnPID.resetPID();
        timeInMillis += System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() <= timeInMillis)) {
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

    //Incorrect version of driveTurn which does not use PID
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

    //Moves armRotator to a specified degree with .setTargetPosition
    protected void armRotateToPosition(double degrees) {
        double startPosition = armRotator.getCurrentPosition();
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator.setTargetPosition((int) Math.round(startPosition + degrees));
    }

    // Task-based Functions

    protected void deploy() {
       int distance = 12650;
       int endpoint = getArmLanderPosition() + distance;
       long timeout = System.currentTimeMillis() + 10000;

       while(distance > 0  && System.currentTimeMillis() < timeout && opModeIsActive()){
           armLander.setPower(((double)distance/500.0) + 0.2);
           distance = endpoint - getArmLanderPosition();
           telemetry.addData("Distance to Ground", distance);
           telemetry.update();
       }
       armLander.setPower(0.0);
       driveStrafePID(3.0, 1600);
       turnRightPID(90.0, 1600);
    }

    protected void sampleDepotSide() {
        double driveToGoldDrive = 42.0;
        double setGrid = 180.0;
        double driveToDepot = 31.0;
        switch (goldTarget)
        {
            case LEFT:
                turnRightPID(-SAMPLE_TURN_ANGLE, 2000);
                driveDistancePID(driveToGoldDrive, 2900);
                turnToFieldPID(setGrid,2900);
                driveDistancePID(driveToDepot, 2900);
                break;
            case RIGHT:
                turnRightPID(SAMPLE_TURN_ANGLE, 2000);
                driveDistancePID(driveToGoldDrive, 2900);
                turnToFieldPID(setGrid, 2900);
                driveStrafePID(-driveToDepot, 2900);
                break;
            case NONE:
            case CENTER:
            default:
                driveDistancePID(52.0, 2500);
                turnToFieldPID(setGrid, 1250);
                break;
        }
    }

    protected void sampleCraterSide() {
        double longDrive = 29.0;
        double shortDrive = 25.25;

        switch (goldTarget)
        {
            case LEFT:
                turnRightPID(-SAMPLE_TURN_ANGLE, 2000);
                driveDistancePID(longDrive, 2000);
                driveDistancePID(-longDrive, 2000);
                break;
            case RIGHT:
                turnRightPID(SAMPLE_TURN_ANGLE, 2000);
                driveDistancePID(longDrive, 2000);
                driveDistancePID(-longDrive, 2000);
                break;
            case NONE:
            case CENTER:
            default:
                driveDistancePID(shortDrive, 2000);
                driveDistancePID(-shortDrive, 2000);
                break;
        }
        turnToFieldPID(176.0, 2000);
    }

    protected void goToDepotCraterSide() {
        driveDistancePID(42.0, 2000);
        turnToFieldPID(180.0, 500);
        driveStrafePID(-68.5, 2000);
    }

    protected void releaseMarker() {
        setMarkerReleaser(1.0);
        sleep(600);
        for(int i=0; i<2; i++){
            setMarkerReleaser(1.0);
            sleep(150);
            setMarkerReleaser(0.5);
            sleep(150);
        }
        setMarkerReleaser(-1.0);
        sleep(300);
    }

    protected void parkInCraterCraterSide() {
        driveStrafePID(65.0, 4000);
        moveTime(0.3, 1000, true, true);
    }

    protected void parkInCraterDepotSide() {
        driveDistancePID(-58.0, 4000);
        driveStrafePID(-12.0, 1000);
        moveTime(0.3, 1000, false, false);
    }
}
