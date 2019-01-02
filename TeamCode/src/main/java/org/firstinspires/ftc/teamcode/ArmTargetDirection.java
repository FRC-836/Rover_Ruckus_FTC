package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ArmTargetDirection {
    // Private
    private static BNO055IMU imu;

    private double pitchAtTargetZero;

    // Constructor, to be used by static functions.
    private ArmTargetDirection(double pitchAtTargetZero) {
        this.pitchAtTargetZero = pitchAtTargetZero;
    }

    // Function to be called before using a TargetDirection object
    public static void setImu(BNO055IMU imu) {
        ArmTargetDirection.imu = imu;
    }

    // Public functions to change directions
    public void moveTargetForwards(double degrees) { pitchAtTargetZero += degrees; }
    public void moveTargetBackwards(double degrees) { pitchAtTargetZero -= degrees; }

    // Calculation functions
    private static double getAbsolutePitch() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return errorCorrecter(angles.thirdAngle);
    }
    private static double calculatePitch() {
        return errorCorrecter(getAbsolutePitch());
    }
    public static double getPitch() { return calculatePitch(); }
    public double calculateDistanceFromTarget() {
        return errorCorrecter(calculatePitch() - pitchAtTargetZero);
    }

    // Functions to create TargetDirection objects
    public static ArmTargetDirection makeTargetAtPitch(double pitchInDegrees) { return new ArmTargetDirection(pitchInDegrees); }
    public static ArmTargetDirection makeTargetToRobotsForward(double degreesToForward) { return new ArmTargetDirection(calculatePitch() + degreesToForward); }
    public static ArmTargetDirection makeTargetToRobotsBackward(double degreesToBackward) { return new ArmTargetDirection(calculatePitch() - degreesToBackward); }

    // Error correction to make everything on the range 0 to +360
    private static double errorCorrecter(double pitch){
        return mod(pitch, 360.0);
    }

    protected static double mod(double a, double b)
    {
        double out = ((a % b) + ((a < 0) ? b : 0));
        return (out >= b) ? 0 : out;
    }
}