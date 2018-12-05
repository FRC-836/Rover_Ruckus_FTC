package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tank Drive", group = "")
public class Tank_Field_Drive extends Teleop_Parent {


    @Override
    public void begin() {
    }

    //Called when teleop begins
    @Override
    public void run() {
        double leftDrivePower = -gamepad1.left_stick_y;
        double rightDrivePower = -gamepad1.right_stick_y;
        double leftStrafePower = gamepad1.left_stick_x;
        double rightStrafePower = gamepad1.right_stick_x;

        //Lowers the speed of drive if driveSlowFactor == true
        if(driveSlowFactor){
            leftDrivePower *= SLOW_DRIVE_SCALE_FACTOR;
            rightDrivePower *= SLOW_DRIVE_SCALE_FACTOR;
            leftStrafePower *= SLOW_DRIVE_SCALE_FACTOR;
            rightStrafePower *= SLOW_DRIVE_SCALE_FACTOR;
        }

        setTankDrive(leftDrivePower, rightDrivePower, leftStrafePower, rightStrafePower);
        //Sets power of armRotator and maps it to joystick controls
        if(gamepad1.right_bumper){
            setArmRotator(LIFT_POWER_UP);
        }
        else if(gamepad1.right_trigger > 0.1f){
            setArmRotator(LIFT_POWER_DOWN);
        }
        else{
            holdArmPosition();
        }

        //Sets power of armExtender and maps it to joystick controls
        if(gamepad1.left_bumper){
            setArmExtender(LIFT_POWER_UP);
        }
        else if(gamepad1.left_trigger > 0.1f){
            setArmExtender(LIFT_POWER_DOWN);
        }
        else{
            setArmExtender(LIFT_POWER_IDLE);
        }

        //Sets power of armLander and maps it to joystick controls
        if(gamepad1.y){
            setArmLander(LIFT_POWER_UP);
        }
        else if(gamepad1.a){
            setArmLander(LIFT_POWER_DOWN);
        }
        else{
            setArmLander(LIFT_POWER_IDLE);
        }

        //Enables or disables slower driving
        if(gamepad1.dpad_up) {
            driveSlowFactor = true;
        }
        else if(gamepad1.dpad_down) {
            driveSlowFactor = false;
        }

        if(gamepad1.x){
            setIntakeMotor(INTAKE_POWER_END);
        }
        else{
            setIntakeMotor(0.0);
        }

        if(gamepad1.dpad_left){
            setIntakeShifter(INTAKE_SHIFTER_POWER);
        }
        else{
            setIntakeShifter(0.0);
        }
    }
    //Sets the power of individual motors to be called based on powers created when run() begins
    private void setTankDrive (double leftDrivePower, double rightDrivePower, double leftStrafePower, double rightStrafePower){
        backLeftDrive.setPower(leftDrivePower + leftStrafePower);
        backRightDrive.setPower(rightDrivePower - rightStrafePower);
        frontLeftDrive.setPower(leftDrivePower - leftStrafePower);
        frontRightDrive.setPower(rightDrivePower + rightStrafePower);
    }
}

