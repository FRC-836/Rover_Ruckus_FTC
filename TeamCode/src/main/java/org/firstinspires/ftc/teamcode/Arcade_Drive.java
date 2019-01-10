package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Arcade Drive")
public class Arcade_Drive extends Teleop_Parent {
    private final double P2_MULT = 0.3;
    private double forwardPower = 0.0;
    private double turnPower = 0.0;
    private double strafePower = 0.0;

    private final double SLOW_TURN_THRESH = (ARM_POSITION_DOWN + ARM_POSITION_UP) / 2.0;
    private final double SLOW_TURN_MULT = 0.3;

    private double lastHeading = 0.0;
    private ElapsedTime lastHeadingTime = new ElapsedTime();
    private final double ANGULAR_VELOCITY_THRESHOLD = 2.0;

    private boolean lastY = false;
    private boolean lastX = false;

    private long lastTime = 0;

    private boolean verboseTiming = true;

    @Override
    public void begin() {
        teleopTurnPID.resetPID();
        currentFacingDirection = TargetDirection.makeTargetToRobotsRight(0.0);
    }

    //Begins teleop
    @Override
    public void run() {
        timeIt("Loop Around");

        setMarkerReleaser(-1.0);
        forwardPower = mapJoyStick(-gamepad1.left_stick_y) + mapJoyStick(gamepad2.left_stick_x) * P2_MULT;
        turnPower = mapJoyStick(gamepad1.right_stick_x) + mapJoyStick(gamepad2.right_stick_x) * P2_MULT;
        strafePower = mapJoyStick(gamepad1.left_stick_x) + mapJoyStick(gamepad2.left_stick_y) * P2_MULT;

        if (getArmRotatorPosition() > SLOW_TURN_THRESH)
            turnPower *= SLOW_TURN_MULT;

        if (Math.abs(strafePower) > Math.abs(forwardPower))
            forwardPower = 0.0;
        else
            strafePower = 0.0;

        //Slows the drive by a certain factor if true
        if (driveSlowFactor) {
            forwardPower *= SLOW_DRIVE_SCALE_FACTOR;
            strafePower *= SLOW_DRIVE_SCALE_FACTOR;
        }

        if (Math.abs(turnPower) < 0.00005f) {
            if (Math.abs(TargetDirection.getHeading() - lastHeading)/lastHeadingTime.milliseconds() < ANGULAR_VELOCITY_THRESHOLD) {
                teleopTurnPID.resetPID();
                teleopTurnPID.update(0.0);
                currentFacingDirection = TargetDirection.makeTargetToRobotsRight(0.0);
            } else {
                turnPower = teleopTurnPID.update(currentFacingDirection.calculateDistanceFromTarget());
            }
        }
        setDrive(forwardPower, turnPower, strafePower);

        if (verboseTiming)
            timeIt("Drive");

        boolean yButton = gamepad1.y;
        boolean xButton = gamepad1.x;

        boolean yIsPressed = yButton && !lastY;
        boolean xIsPressed = xButton && !lastX;

        lastY = yButton;
        lastX = xButton;


        //Lifts the arm to certain positions and maps them to certain joystick positions
        if (gamepad1.left_bumper) {
            setArmRotatorGoal(ARM_ROTATOR_POWER_UP);
        } else if (gamepad1.left_trigger > 0.1f) {
            setArmRotatorGoal(ARM_ROTATOR_POWER_DOWN);
        } else if (yIsPressed) { // Up
            armHasBeenHolding = false;
            useP = true;
            holdArmPosition(ARM_POSITION_UP);
        } else if (xIsPressed) { // Center
            armHasBeenHolding = false;
            useP = true;
            holdArmPosition(ARM_POSITION_DOWN);
        } else {
            holdArmPosition();
        }

        if (verboseTiming)
            timeIt("Arm Rotator");

        //Extends the arm to certain positions, and maps them to certain joystick positions
        if (gamepad1.right_bumper) {
            setArmExtender(ARM_EXTENDER_POWER_UP);
        } else if (gamepad1.right_trigger > 0.1f) {
            setArmExtender(ARM_EXTENDER_POWER_DOWN);
        } else {
            setArmExtender(ARM_EXTENDER_POWER_IDLE);
        }

        //Set lander to certain positions, and maps them to certain joystick positions
        if (gamepad2.y) {
            setArmLander(ARM_LANDER_POWER_UP);
        } else if ((gamepad2.a) && (!gamepad1.start && !gamepad2.start)) {
            setArmLander(ARM_LANDER_POWER_DOWN);
        } else {
            setArmLander(ARM_LANDER_POWER_IDLE);
        }

        if (verboseTiming)
            timeIt("Arm Extender and Lander");

        //Enables or disables a slower drive
        if (gamepad1.dpad_left) {
            driveSlowFactor = true;
        } else if (gamepad1.dpad_right) {
            driveSlowFactor = false;
        }

        if (gamepad2.left_trigger > 0.1f) {
            setIntakeMotor(INTAKE_POWER_END);
        } else if (gamepad2.left_bumper) {
            setIntakeMotor(-INTAKE_POWER_END);
        } else {
            setIntakeMotor(0.0);
        }

        if (verboseTiming)
            timeIt("Slow Drive and Intake");

        telemetry.addData("Arm Position", getArmRotatorPosition());
        telemetry.addData("Arm Motor Power", armRotator.getPower());
        telemetry.addData("Setpoint", armHoldP.getSetpoint());
        telemetry.update();
    }

    private void timeIt(String message)
    {
        long time = System.currentTimeMillis();
        telemetry.addData(message,time - lastTime);
        lastTime = time;
    }
}