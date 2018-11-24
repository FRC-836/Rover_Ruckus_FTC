package org.firstinspires.ftc.teamcode;

public abstract class Teleop_Parent extends Robot_Parent {
    //Final and boolean powers which are used to set powers of specific parts of the robot
    boolean driveSlowFactor = true;
    final double SLOW_DRIVE_SCALE_FACTOR = 0.5;
    final double LIFT_POWER_UP = 0.5;
    final double LIFT_POWER_DOWN = -0.5;
    final double LIFT_POWER_IDLE = 0.0;

    //Called on init
    @Override
    public void getReady() {

    }

    //Called on start, does actions when the robot is running
    @Override
    public void go() {
        begin();
        while (opModeIsActive()) {
            run();
        }
    }

    abstract public void begin();

    abstract public void run();

    //Transfers float inputs of joystick values to doubles
    protected double mapJoyStick(float joyStickImput){
        return (double)joyStickImput;
    }
}
