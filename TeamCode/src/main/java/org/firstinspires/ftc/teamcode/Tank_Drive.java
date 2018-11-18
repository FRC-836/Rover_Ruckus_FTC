package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tank drive", group = "")
public class Tank_Drive extends Teleop_Parent {

    @Override
    public void begin() {
    }

    @Override
    public void run() {
        double leftDrivePower = -gamepad1.left_stick_y;
        double rightDrivePower = -gamepad1.right_stick_y;
        double leftStrafePower = gamepad1.left_stick_x;
        double rightStrafePower = gamepad1.right_stick_x;

        if(driveSlowFactor){
            leftDrivePower *= SLOW_DRIVE_SCALE_FACTOR;
            rightDrivePower *= SLOW_DRIVE_SCALE_FACTOR;
            leftStrafePower *= SLOW_DRIVE_SCALE_FACTOR;
            rightStrafePower *= SLOW_DRIVE_SCALE_FACTOR;
        }

        setTankDrive(leftDrivePower, rightDrivePower, leftStrafePower, rightStrafePower);
    }
    private void setTankDrive (double leftDrivePower, double rightDrivePower, double leftStrafePower, double rightStrafePower){
        backLeftDrive.setPower(leftDrivePower + leftStrafePower);
        backRightDrive.setPower(rightDrivePower - rightStrafePower);
        frontLeftDrive.setPower(leftDrivePower - leftStrafePower);
        frontRightDrive.setPower(rightDrivePower + rightStrafePower);
    }
}

