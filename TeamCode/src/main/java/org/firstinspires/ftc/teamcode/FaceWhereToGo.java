package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Face Where to Go")
public class FaceWhereToGo extends Teleop_Parent {

    //Team is using Regular/H drive train
    Heading heading;
    boolean isTurning = false;
    boolean dpadPressed = false;
    boolean isStopped = false;

    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        heading = Heading.createRelativeHeading(0.0f);
        holdTurnPID.resetPID();
        holdTurnPID.setSetpoint(0.0);
    }

    @Override
    public void repeat() {
        // Get powers
        double away = -gamepad1.left_stick_y;
        double side = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;

        if (Math.abs(away) < JOYSTICK_DEAD_ZONE && Math.abs(side) < JOYSTICK_DEAD_ZONE
                && Math.abs(strafe) < JOYSTICK_DEAD_ZONE) {
            setDrive(0.00, 0.00, 0.00);
            holdTurnPID.update(heading.getRelativeHeading());
        } else {

            double directionOfJoystick = Math.toDegrees(Math.atan2(side,away));
            double inputSize = Math.sqrt(Math.pow(away, 2) + Math.pow(side, 2));

            // If user is trying to drive around
            if (Math.abs(away) > JOYSTICK_DEAD_ZONE || Math.abs(away) > JOYSTICK_DEAD_ZONE)
                heading.setRelativeOffset((float)-directionOfJoystick);

            double pidTurnPower = holdTurnPID.update(heading.getRelativeHeading());
            setDrive(inputSize, pidTurnPower, strafe);

            telemetry.addData("pgain", p);
            telemetry.addData("dgain", d);
            telemetry.addData("Joystick Direction", directionOfJoystick);
            telemetry.addData("Joystick Power", inputSize);
            telemetry.update();
        }
    }

}