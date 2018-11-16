package org.firstinspires.ftc.teamcode;

public abstract class Teleop_Parent extends Robot_Parent {

    protected static final float JOYSTICK_DEAD_ZONE = 0.1f;

    @Override
    public void initialize() {
        gamepad1.setJoystickDeadzone(JOYSTICK_DEAD_ZONE);
        setup();
    }

    @Override
    public void play() {
        begin();

        while (opModeIsActive()) {
            repeat();
        }
    }

    public abstract void repeat();

    protected double mapJoystick(double joystickValue) {
        return joystickValue;
    }
}
