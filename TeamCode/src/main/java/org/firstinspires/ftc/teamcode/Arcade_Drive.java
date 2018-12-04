package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Arcade Drive", group = "")
public class Arcade_Drive extends Teleop_Parent {
    private final double P2_MULT = 0.5;
    private boolean holdingTurn = false;
    private PID_Controller tHoldTurnPID = new PID_Controller(0.012, 0.0, 0.0013);
    TargetDirection currentTarget;

    @Override
    public void begin() {
        tHoldTurnPID.setSetpoint(0.0);
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

        if(Math.abs(turnPower) < 0.005f) {
            if (!holdingTurn) {
                currentTarget = TargetDirection.makeTargetToRobotsRight(0.0);
                tHoldTurnPID.resetPID();
                tHoldTurnPID.update(currentTarget.calculateDistanceFromTarget());
                holdingTurn = true;
                sleep(10);
            }
            turnPower = tHoldTurnPID.update(currentTarget.calculateDistanceFromTarget());
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
        if(gamepad1.y){
            setArmLander(LIFT_POWER_UP);
        }
        else if(gamepad1.a){
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
