package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Test")
public class Autonomous_Test extends Autonomous_Parent {

    private final long sleepTime = 2000;

    @Override
    public void setup() {
    }

    @Override
    public void begin() {
        // TODO: find why all turns after first turn fail
        driveDistance(36);
        turn (90);
        driveDistance(36);
        turn(-90);
        driveDistance(-36);
        turn(-90);
        driveDistance(36);
        turn(90);
    }

    public void driveDistance(double inches) {
        double goal = getForwardPosition() + inches;

        double multiplier = 1.0;
        if (inches < 0.0)
            multiplier = -1.0;

        setDrive(multiplier, 0.0, 0.0);
        while (multiplier * (getForwardPosition() - goal) < 0.0) {
            setDrive(multiplier, 0.0, 0.0);
        }
        setDrive(0.0, 0.0, 0.0);
        sleep(sleepTime);
    }

    public void turn(double degrees) {
        Heading goalHeading = Heading.createRelativeHeading((float) -degrees);
        double multiplier = 1.0;
        if (degrees < 0.0)
            multiplier = -1.0;

        telemetry.addData("Degrees", degrees);
        telemetry.addData("Multiplier", multiplier);
        telemetry.addData("Current Relative Heading", goalHeading.getRelativeHeading());
        telemetry.update();
        sleep(3000);

        setDrive(0.0, multiplier, 0.0);
        while (multiplier * goalHeading.getRelativeHeading() < 0.0) {
            setDrive(0.0, multiplier, 0.0);
        }
        setDrive(0.0, 0.0, 0.0);
        sleep(sleepTime);
    }
}