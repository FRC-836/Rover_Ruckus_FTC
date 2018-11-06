package org.firstinspires.ftc.teamcode;

public class Tank_Drive extends Teleop_Parent {

    @Override
    public void begin() {
    }

    @Override
    public void run() {
        double leftDrivePower = -gamepad1.left_stick_y;
        double rightDrivePower = -gamepad1.left_stick_y;
        double leftStrafePower = gamepad1.left_stick_x;
        double rightStrafePower = gamepad1.right_stick_x;

        setTankDrive(leftDrivePower, rightDrivePower, rightStrafePower, leftStrafePower);
    }
    private void setTankDrive (double leftDrivePower, double rightDrivePower, double leftStrafePower, double rightStrafePower){
        backLeftDrive.setPower(leftDrivePower - leftStrafePower);
        backRightDrive.setPower(rightDrivePower + rightStrafePower);
        frontLeftDrive.setPower(leftDrivePower + leftStrafePower);
        frontRightDrive.setPower(rightDrivePower - rightStrafePower);
    }
}

