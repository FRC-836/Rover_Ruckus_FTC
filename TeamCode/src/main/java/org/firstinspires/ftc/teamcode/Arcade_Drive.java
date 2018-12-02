package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Arcade Drive", group = "")
public class Arcade_Drive extends Teleop_Parent {
    @Override
    public void begin() {

    }

    //Begins teleop
    @Override
    public void run() {
        double forwardPower = -gamepad1.left_stick_y;
        double turnPower = gamepad1.right_stick_x;
        double strafePower = gamepad1.left_stick_x;

        //Slows the drive by a certain factor if true
        if (driveSlowFactor)
        {
            forwardPower *= SLOW_DRIVE_SCALE_FACTOR;
            strafePower *= SLOW_DRIVE_SCALE_FACTOR;
        }
        setDrive(forwardPower, strafePower, turnPower);

        //Lifts the arm to certain positions and maps them to certain joystick positions
        if(gamepad1.left_bumper){
            setArmRotator(LIFT_POWER_UP);
        }
        else if(gamepad1.right_bumper){
            setArmRotator(LIFT_POWER_DOWN);
        }
        else{
            setArmRotator(LIFT_POWER_IDLE);
        }

        //Extends the arm to certain positions, and maps them to certain joystick positions
        if(gamepad1.left_trigger > 0.1f){
            setArmExtender(LIFT_POWER_UP);
        }
        else if(gamepad1.right_trigger > 0.1f){
            setArmExtender(LIFT_POWER_DOWN);
        }
        else{
            setArmExtender(0.0);
        }

        //Set lander to certain positions, and maps them to certain joystick positions
        if(gamepad1.y){
            setArmLander(LIFT_POWER_UP);
        }
        else if(gamepad1.b){
            setArmLander(LIFT_POWER_DOWN);
        }
        else{
            setArmLander(LIFT_POWER_IDLE);
        }

        //Enables or disables a slower drive
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
}
