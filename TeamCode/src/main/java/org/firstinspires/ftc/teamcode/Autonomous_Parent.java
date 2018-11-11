package org.firstinspires.ftc.teamcode;

public abstract class Autonomous_Parent extends Robot_Parent {

    // TODO: remember to set values to diff number
    private final double ENCODER_COUNTS_PER_INCH = 81.19;

<<<<<<< HEAD
    protected PID_Controller forwardPID = new PID_Controller(0.071, 0.0, 0.0);
    protected PID_Controller strafePID = new PID_Controller(0.071, 0.0, 0.0);
=======
    protected PID_Controller forwardPID = new PID_Controller(0.071,0.0,0.0);
>>>>>>> 12888_dev

    @Override
    public void initialize() {
        setup();
    }

    @Override
    public void play() {
        begin();
    }

    protected double getForwardPosition() {
        double position;
        position = backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition() + frontLeftDrive.getCurrentPosition() + frontLeftDrive.getCurrentPosition();
        position /= 4.0;

        position /= ENCODER_COUNTS_PER_INCH;

        return position;
    }
}