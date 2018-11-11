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
    
    protected double getStrafePosition() {
        double position = -backLeftDrive.getCurrentPosition() +
                frontLeftDrive.getCurrentPosition() - frontRightDrive.getCurrentPosition() +
                backRightDrive.getCurrentPosition();

        position /= 4.0;
        position /= ENCODER_COUNTS_PER_INCH;

        return position;
    }
    protected void land(){
        // TODO: Landing Mechanism Needed
    }
    protected void driveStraight(double inches) {
        double startPosition = getForwardPosition();
        while(opModeIsActive() && Math.abs(getForwardPosition() - startPosition) < inches) {
            if (inches > 0.0) {
                setDrive(1.0, 0.0, 0.0);
            }
            else {
                setDrive(-1.0, 0.0, 0.0);
            }
        }
        setDrive(0.0,0.0,0.0);
    }
    protected void turnDegrees(double degrees) {
        double startPosition = getTurnPosition();
        while (opModeIsActive() && Math.abs(getTurnPosition() - startPosition) < degrees) {
            if (degrees > 0.0) {
                setDrive(0.0,1.0,0.0);
            }
            else {
                setDrive(0.0,-1.0,0.0);
            }
        }
        setDrive(0.0,0.0,0.0);
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
    protected void scoreMineralInDepot(){
        //TODO: Insert Scoring code in depot (Mechanism needed)
    }
    protected void driveToDepot(boolean sample, boolean collect) {
        if (sample) {
            driveStraight(1.5);
            sample();
            driveStraight(1.0);
        }
        else if (collect) {
            driveStraight(1.5);
            collectMineral();
            driveStraight(1.0);
        }
            else{
           driveStraight(2.5);
        }
    }
    protected void driveFromCraterToDepot(boolean driveDirectlyRight) {
        land();
        driveStraight(1.0);
        sample();
        if(driveDirectlyRight) {
            turnDegrees(-70.0);
            driveStraight(1.0);
            turnDegrees(-35.0);
            driveStraight(2.8);
        }
        else {
            turnDegrees(-175.0);
            driveStraight(1.3);
            turnDegrees(40.0);
            driveStraight(1.6);
            turnDegrees(-90.0);
            driveStraight(1.4);
        }
    }
    protected void driveFromDepotToPark(boolean driveDirectlyLeft) {
        land();
        //TODO Change ALL values; These are estimates!
        if (driveDirectlyLeft){
            turnDegrees(35.0);
            driveStraight(3.0);
        }
        else {
            turnDegrees(180.0);
            driveStraight(2.5);
            turnDegrees(-45.0);
            driveStraight(1.2);
            turnDegrees(-90.0);
            driveStraight(1.7);
            turnDegrees(90.0);
            driveStraight(0.5);
        }
    }
}