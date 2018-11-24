package org.firstinspires.ftc.teamcode;

public abstract class Teleop_Parent extends Robot_Parent {
    boolean driveSlowFactor = true;
    final double SLOW_DRIVE_SCALE_FACTOR = 0.5;
    final double LIFT_POWER_UP = 0.5;
    final double LIFT_POWER_DOWN = -0.5;
    final double LIFT_POWER_IDLE = 0.3;

    @Override
    public void getReady() {

    }

    @Override
    public void go() {
        begin();
        while (opModeIsActive()) {
            run();
        }
    }

    abstract public void begin();

    abstract public void run();

    protected double mapJoyStick(float joyStickImput){
        return (double)joyStickImput;
    }
}
