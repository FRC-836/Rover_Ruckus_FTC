package org.firstinspires.ftc.Interfaces;

public interface DriveInterface extends SubAssembly{
    void setDrive(double leftPower, double rightPower);

    void driveDistance(double inches);
    void turnDegrees(double degrees);

    boolean isDoneMoving();
}
