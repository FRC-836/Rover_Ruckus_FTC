package org.firstinspires.ftc.Interfaces;

public interface DriveInterface extends SubAssembly{
    void setDrive(double forwardPower, double turnPower);

    void driveDistance(double inches);
    void turnDegrees(double degrees);

    boolean isDoneMoving();
}
