package org.firstinspires.ftc.parent_classes;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Teleop_Parent extends Robot_Parent {

    protected static final float JOYSTICK_DEAD_ZONE = 0.1f;

    @Override
    public void initialize() {
        intakeLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        end();
    }

    public abstract void repeat();
    public abstract void end();

    protected double mapJoystick(double joystickValue) {
        return joystickValue;
    }

    public void setInOut(double inOutPower) {
        intakeInOut.setPower(inOutPower);
    }
}
