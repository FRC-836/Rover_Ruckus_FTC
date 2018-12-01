package org.firstinspires.ftc.parent_classes;

public abstract class Teleop_Parent extends Robot_Parent {

    protected static final float JOYSTICK_DEAD_ZONE = 0.1f;

    @Override
    public void initialize() {
        isAuto = false;
        gamepad1.setJoystickDeadzone(JOYSTICK_DEAD_ZONE);
        setup();
        retractTeamMarkerServo();
    }

    @Override
    public void play() {
        begin();
        retractTeamMarkerServo();

        while (opModeIsActive()) {
            repeat();
        }
    }

    public abstract void repeat();

    protected double mapJoystick(double joystickValue) {
        return joystickValue;
    }
}
