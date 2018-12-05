package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Arcade Field Centric", group = "")
public class Arcade_Field_Centric extends Teleop_Parent {
    private final double P2_MULT = 0.5;
    private boolean holdingTurn = false;

    @Override
    public void begin() {
    }

    //Begins teleop
    @Override
    public void run() {
        double forwardPower = -gamepad1.left_stick_y + gamepad2.left_stick_x*P2_MULT;
        double turnPower = gamepad1.right_stick_x + gamepad2.right_stick_x*P2_MULT;
        double strafePower = gamepad1.left_stick_x + gamepad2.left_stick_y*P2_MULT;

        if (Math.abs(strafePower) > Math.abs(forwardPower))
            forwardPower = 0.0;
        else
            strafePower = 0.0;

        //Slows the drive by a certain factor if true
        if (driveSlowFactor) {
            forwardPower *= SLOW_DRIVE_SCALE_FACTOR;
            strafePower *= SLOW_DRIVE_SCALE_FACTOR;
        }

        if(Math.abs(turnPower) < 0.00005f) {
            if (!holdingTurn) {
                setDrive(forwardPower, 0.0, strafePower);
                teleopTurnPID.resetPID();
                teleopTurnPID.update(0.0);
                sleep(100);
                teleopTurnPID.update(0.0);
                currentFacingDirection = TargetDirection.makeTargetToRobotsRight(0.0);
                holdingTurn = true;
            } else {
                turnPower = teleopTurnPID.update(currentFacingDirection.calculateDistanceFromTarget());
            }
        }
        else
        {
            holdingTurn = false;
        }
        setDrive(forwardPower, turnPower, strafePower);

        //Lifts the arm to certain positions and maps them to certain joystick positions
        if(gamepad1.right_bumper){
            setArmRotatorGoal(LIFT_POWER_UP);
        }
        else if(gamepad1.right_trigger > 0.1f){
            setArmRotatorGoal(LIFT_POWER_DOWN);
        }
        else{
            holdArmPosition();
        }

        //Extends the arm to certain positions, and maps them to certain joystick positions
        if(gamepad1.left_bumper){
            setArmExtender(LIFT_POWER_UP);
        }
        else if(gamepad1.left_trigger > 0.1f){
            setArmExtender(LIFT_POWER_DOWN);
        }
        else{
            setArmExtender(0.0);
        }

        //Set lander to certain positions, and maps them to certain joystick positions
        if(gamepad1.y || gamepad2.y){
            setArmLander(LIFT_POWER_UP);
        }
        else if((gamepad1.a || gamepad2.a) && (!gamepad1.start && !gamepad2.start)){
            setArmLander(LIFT_POWER_DOWN);
        }
        else{
            setArmLander(LIFT_POWER_IDLE);
        }

        //Enables or disables a slower drive
        if(gamepad1.dpad_down) {
            driveSlowFactor = true;
        }
        else if(gamepad1.dpad_up) {
            driveSlowFactor = false;
        }

        if(gamepad1.x){
            setIntakeMotor(INTAKE_POWER_END);
        } else if(gamepad1.b) {
            setIntakeMotor(-INTAKE_POWER_END);
        }else{
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
