package org.firstinspires.ftc.teamcode;

public abstract class Teleop_Parent extends Robot_Parent {

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
