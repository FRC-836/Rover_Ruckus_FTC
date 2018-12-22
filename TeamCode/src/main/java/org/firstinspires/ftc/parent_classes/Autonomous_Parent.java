package org.firstinspires.ftc.parent_classes;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.teamcode.PID_Controller;
import org.firstinspires.ftc.teamcode.TargetDirection;

public abstract class Autonomous_Parent extends Robot_Parent {

    private final double ENCODER_COUNTS_PER_INCH = 81.19;
    private final double EARLY_STOP_DEGREES = 5.0;
    private final double MILLIS_PER_INCH = 27.7;
    private final double SAMPLE_TURN_ANGLE = 27.5;

    protected Sampler sampler = new Sampler();
    protected Sampler.GoldPosition position = Sampler.GoldPosition.UNKNOWN;

    protected PID_Controller forwardPID = new PID_Controller(0.071, 0.0, 0.0);
    protected TargetDirection lastTurnDirection;

    @Override
    public void initialize() {
        isAuto = true;
        setup();
        setServoIntake(1.0);
        sleep(500);
        setServoIntake(0.0);
        sampler.init(telemetry, hardwareMap);
    }

    @Override
    public void play() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        Sampler.GoldPosition pos;
        while (timer.seconds() < 1.8 && opModeIsActive())
        {
            pos = sampler.sample();
            if (pos != Sampler.GoldPosition.UNKNOWN)
                position = pos;
        }
        begin();
    }

    protected double getForwardPosition() {
        double position;
        position = backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition();
        position /= 2.0;

        position /= ENCODER_COUNTS_PER_INCH;

        return position;
    }

    protected int getLandingMotorPosition() {
        return landingMotor.getCurrentPosition();
    }

    protected void land() {
        setLandingMotorPower(DEPLOY_POWER);
        sleep(2500);
        setArcadeDrive(-0.13,0.0);
        sleep(750);
        setArcadeDrive(0.0,0.0);
        setLandingMotorPower(-DEPLOY_POWER);
        sleep(2000);
        driveDistanceTime(3.0);
        setLandingMotorPower(0.0);
    }

    protected void sampleDepot() {
        // must set intakeMotor for sampling, and correclty set intakeCRServo
        switch (position) {
            case LEFT:
                driveDistanceTime(8.0);
                telemetry.addLine("Left");
                telemetry.update();
                sleep(1000);
                turnDegreesPID(-SAMPLE_TURN_ANGLE, 2000);
                //setServoIntake(1.0);
                driveDistanceTime(44.0, lastTurnDirection);
                //setServoIntake(0.0);
                turnDegreesPID(45.0 + SAMPLE_TURN_ANGLE, 4000);
                driveDistanceTime(30.0, lastTurnDirection);
                placeTeamMarker();
                turnPID(TargetDirection.makeTargetAtFieldPosition(180.0), 3000);
                break;
            case RIGHT:
                driveDistanceTime(8.0);
                telemetry.addLine("Right");
                telemetry.update();
                sleep(2000);
                turnDegreesPID(SAMPLE_TURN_ANGLE, 2000);
               // setServoIntake(1.0);
                driveDistanceTime(44.0, lastTurnDirection);
                //setServoIntake(0.0);
                turnDegreesPID(-45.0 - SAMPLE_TURN_ANGLE, 4000);
                driveDistanceTime(30.0, lastTurnDirection);
                placeTeamMarker();
                turnPID(TargetDirection.makeTargetAtFieldPosition(180.0), 5000);
                break;
            default:
                //setServoIntake(1.0);
                driveDistanceTime(78.0);
                //setServoIntake(0.0);
                turnPID(TargetDirection.makeTargetAtFieldPosition(180.0), 4000);
                placeTeamMarker();
                break;
        }
    }

    protected void sampleCrater() {
        // must set intakeMotor for sampling, and correclty set intakeCRServo
        double centerDrive = 33.5;
        switch (position) {
            case LEFT:
                turnDegreesPID(-SAMPLE_TURN_ANGLE, 2200);
                initIntake();
                driveDistanceTime(36.0, lastTurnDirection);
                stopIntake();
                driveDistanceTime(-36.0, lastTurnDirection);
                turnPID(TargetDirection.makeTargetAtFieldPosition(170.0), 1200);
                break;
            case RIGHT:
                turnDegreesPID(SAMPLE_TURN_ANGLE, 2200);
                initIntake();
                driveDistanceTime(36.0, lastTurnDirection);
                stopIntake();
                driveDistanceTime(-36.0, lastTurnDirection);
                turnPID(TargetDirection.makeTargetAtFieldPosition(170.0), 2200);
                break;
            default:
                initIntake();
                driveDistanceTime(centerDrive);
                stopIntake();
                driveDistanceTime(-centerDrive);
                turnPID(TargetDirection.makeTargetAtFieldPosition(170.0), 1800);
                break;
        }
    }

    protected void sampleParkCrater() {
        switch (position) {
            case LEFT:
                turnDegreesPID(-27.5, 2000);
                initIntake();
                driveDistanceTime(40.0);
                stopIntake();
                break;
            case RIGHT:
                turnDegreesPID(27.5, 2000);
                initIntake();
                driveDistanceTime(40.0);
                stopIntake();
                break;
            default:
                initIntake();
                driveDistanceTime(40.0);
                stopIntake();
                break;
        }
    }

    protected void parkDepot() {
        driveDistanceTime(-60.0, lastTurnDirection);
    }

    protected void parkCrater() {
        turnPID(TargetDirection.makeTargetAtFieldPosition(90.0), 1800);
        driveDistanceTime(-88.0, lastTurnDirection);
    }

    protected void placeTeamMarker() {
        dropTeamMarker();
        sleep(1000);
        retractTeamMarkerServo();
    }

    protected void driveToDepot() {
        driveDistanceTime(63.0, lastTurnDirection);
        turnPID(TargetDirection.makeTargetAtFieldPosition(93.0), 2000);
        driveDistanceTime(65.0, lastTurnDirection);
        turnPID(TargetDirection.makeTargetAtFieldPosition(135.0), 1500);
    }

    protected void driveDistanceTime(double inches) {
        driveDistanceTime(inches, TargetDirection.makeTargetToRobotsRight(0.0));
    }

    protected void driveDistanceTime(double inches, TargetDirection target) {
        long endTime = System.currentTimeMillis() + Math.round(MILLIS_PER_INCH * Math.abs(inches));

        goToTurnPID.setSetpoint(0.0);
        goToTurnPID.resetPID();
        while (System.currentTimeMillis() < endTime && opModeIsActive())
        {
            double turnPower = goToTurnPID.update(target.calculateDistanceFromTarget());
            if (inches > 0.0)
                setArcadeDrive(1.0, turnPower);
            else
                setArcadeDrive(-1.0, turnPower);
        }

        setArcadeDrive(0.0, 0.0);
        sleep(500);
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

    protected double turnPID(TargetDirection target, long millis) {
        long endTime = System.currentTimeMillis() + millis;
        goToTurnPID.setSetpoint(0.0);
        goToTurnPID.resetPID();
        while (System.currentTimeMillis() < endTime && opModeIsActive()) {
            double turnPower = goToTurnPID.update(target.calculateDistanceFromTarget());
            setArcadeDrive(0.0, turnPower);
        }
        setArcadeDrive(0.0, 0.0);
        lastTurnDirection = target;
        return target.calculateDistanceFromTarget();
    }

    protected double turnDegreesPID(double degrees, long millis) {
        TargetDirection target = TargetDirection.makeTargetToRobotsRight(degrees);
        return turnPID(target, millis);
    }

    protected void initIntake() {
        setServoIntake(1.0);
        sleep(600);
        setServoIntake(0.0);
        setMotorIntake(1.0);
    }

    protected  void stopIntake() {
        setServoIntake(-1.0);
        sleep(600);
        setMotorIntake(0.0);
    }
}