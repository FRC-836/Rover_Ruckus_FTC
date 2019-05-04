package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class TargetDirection {
    // Private Variables
    private static double absoluteHeadingAtFieldZero;
    private static BNO055IMU imu;

    private double fieldHeadingAtTargetZero;

    // Constructor, to be used by static functions.
    private TargetDirection(double fieldHeadingAtTargetZero) {
        this.fieldHeadingAtTargetZero = fieldHeadingAtTargetZero;
    }

    // Function to be called before using a TargetDirection object
    public static void setImu(BNO055IMU imu) {
        TargetDirection.imu = imu;
    }

    public static void setCurrentHeading(double robotsCurrentHeading){
        TargetDirection.absoluteHeadingAtFieldZero = getAbsoluteHeading() - robotsCurrentHeading;
    }

    // Public functions to change directions
    public void moveTargetToRight(double degrees) { fieldHeadingAtTargetZero += degrees; }
    public void moveTargetToLeft(double degrees) { fieldHeadingAtTargetZero -= degrees; }
    public void setToFieldDirection(double degrees) { this.fieldHeadingAtTargetZero = degrees; }

    // Calculation functions
    private static double getAbsoluteHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return errorCorrecter(-angles.firstAngle);
    }
    private static double calculateFieldHeading() {
        return errorCorrecter(getAbsoluteHeading() - absoluteHeadingAtFieldZero);
    }
    public static double getHeading() { return calculateFieldHeading(); }
    public double calculateDistanceFromTarget() {
        return errorCorrecter(calculateFieldHeading() - fieldHeadingAtTargetZero);
    }

    // Functions to create TargetDirection objects
    public static TargetDirection makeTargetAtFieldPosition(double fieldPositionInDegrees) { return new TargetDirection(fieldPositionInDegrees); }
    public static TargetDirection makeTargetToRobotsRight(double degreesToRight) { return new TargetDirection(calculateFieldHeading() + degreesToRight); }
    public static TargetDirection makeTargetToRobotsLeft(double degreesToLeft) { return new TargetDirection(calculateFieldHeading() - degreesToLeft); }

    // Error correction to make everything on the range -180 to +180
    private static double errorCorrecter(double heading){
        if (heading > 180f)
            heading = ((heading + 180f) % 360f) - 180f;
        else if (heading < -180f)
            heading = 180f - ((180f - heading) % 360f);
        return heading;
    }
}
