package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Face Where to Go")
public class FaceWhereToGo extends Teleop_Parent {

    //Team is using Regular/H drive train
    Heading heading;
    boolean resetPid = true;

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
        double awayDirectional = -gamepad1.left_stick_y;
        double sideDirectional = gamepad1.left_stick_x;
        double awayStraight = -gamepad1.right_stick_y;
        double sideStraight = gamepad1.right_stick_x;

        boolean directional = Math.abs(awayDirectional) > JOYSTICK_DEAD_ZONE || Math.abs(sideDirectional) > JOYSTICK_DEAD_ZONE;
        boolean straight = Math.abs(awayStraight) > JOYSTICK_DEAD_ZONE || Math.abs(sideStraight) > JOYSTICK_DEAD_ZONE;

        if (!directional && !straight) {
            setDrive(0.00, 0.00, 0.00);
            holdTurnPID.update(heading.getRelativeHeading());
            resetPid = true;
        } else if (directional) {
            double directionOfJoystick = Math.toDegrees(Math.atan2(sideDirectional,awayDirectional));
            double inputSize = Math.sqrt(Math.pow(awayDirectional, 2) + Math.pow(sideDirectional, 2));

            heading.setRelativeOffset((float)-directionOfJoystick);
            double pidTurnPower = holdTurnPID.update(heading.getRelativeHeading());

            setDrive(inputSize, pidTurnPower, 0.0);

            telemetry.addData("pgain", p);
            telemetry.addData("dgain", d);
            telemetry.addData("Joystick Direction", directionOfJoystick);
            telemetry.addData("Joystick Power", inputSize);
            telemetry.update();
            resetPid = true;
        } else { // Straight
            double directionOfJoystick = Math.toDegrees(Math.atan2(sideStraight,awayStraight));
            double inputSize = Math.sqrt(Math.pow(sideDirectional, 2) + Math.pow(sideStraight, 2));

            double fieldHeading = Heading.getFieldHeading();
            double robotOrientedCommand = directionOfJoystick - fieldHeading;
            double strafe = inputSize * Math.sin(Math.toRadians(robotOrientedCommand));
            double forward = inputSize * Math.cos(Math.toRadians(robotOrientedCommand));

            if (resetPid)
            {
                heading.setRelativeOffset((float)-fieldHeading);
                resetPid = false;
            }

            double pidTurnPower = holdTurnPID.update(heading.getRelativeHeading());

            setDrive(forward, pidTurnPower, strafe);

            telemetry.addData("pgain", p);
            telemetry.addData("dgain", d);
            telemetry.addData("Joystick Direction", directionOfJoystick);
            telemetry.addData("Joystick Power", inputSize);
            telemetry.update();
        }
    }

}