package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ArmTargetDirection {
    // Private Variables
    private static double absolutePitchAtZeroPitch;
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

    public static void setCurrentPitch(double armsCurrentPitch){
        ArmTargetDirection.absolutePitchAtZeroPitch = getAbsolutePitch() - armsCurrentPitch;
    }

    // Public functions to change directions
    public void moveTargetForwards(double degrees) { pitchAtTargetZero += degrees; }
    public void moveTargetBackwards(double degrees) { pitchAtTargetZero -= degrees; }

    // Calculation functions
    private static double getAbsolutePitch() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return errorCorrecter(-angles.thirdAngle);
    }
    private static double calculatePitch() {
        return errorCorrecter(getAbsolutePitch() - absolutePitchAtZeroPitch);
    }
    public static double getPitch() { return calculatePitch(); }
    public double calculateDistanceFromTarget() {
        return errorCorrecter(calculatePitch() - pitchAtTargetZero);
    }

    // Functions to create TargetDirection objects
    public static ArmTargetDirection makeTargetAtPitch(double pitchInDegrees) { return new ArmTargetDirection(pitchInDegrees); }
    public static ArmTargetDirection makeTargetToRobotsForward(double degreesToForward) { return new ArmTargetDirection(calculatePitch() + degreesToForward); }
    public static ArmTargetDirection makeTargetToRobotsBackward(double degreesToBackward) { return new ArmTargetDirection(calculatePitch() - degreesToBackward); }

    // Error correction to make everything on the range -180 to +180
    private static double errorCorrecter(double pitch){
        if (pitch > 180f)
            pitch = ((pitch + 180f) % 360f) - 180f;
        else if (pitch < -180f)
            pitch = 180f - ((180f - pitch) % 360f);
        return pitch;
    }
}
