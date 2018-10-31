package org.firstinspires.ftc.teamcode;

public abstract class Teleop_Parent extends Robot_Parent {

    @Override
    public void initializeRobot() {

    }

    @Override
    public void startRobot() {
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
