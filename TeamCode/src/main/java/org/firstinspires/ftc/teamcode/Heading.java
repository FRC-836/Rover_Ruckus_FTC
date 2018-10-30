package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Heading {
    private static float fieldOffset;
    private static BNO055IMU imu;

    private float relativeOffset;

    Heading(float relativeOffset) {
        this.relativeOffset = relativeOffset;
    }

    public static float getFieldOffset(){
        return fieldOffset;
    }
    public static void setFieldOffset(float fieldOffset){
        Heading.fieldOffset = fieldOffset;
    }
    public static void setImu(BNO055IMU imu) {
        Heading.imu = imu;
    }
    public static float getAbsoluteHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return errorCorrecter(-angles.firstAngle);
    }
    public static float getFieldHeading(){
        return errorCorrecter(getAbsoluteHeading() + fieldOffset);
    }
    private static float errorCorrecter(float sumOfHeadings){
        if (sumOfHeadings > 180f){
            sumOfHeadings = ((sumOfHeadings + 180f) % 360f) - 180f;
        }
        else if (sumOfHeadings < -180f) {
            sumOfHeadings = 180f - ((180f - sumOfHeadings) % 360f);
        }
        return sumOfHeadings;
    }
    public static Heading createFieldHeading(float relativeOffset) {
        return new Heading(relativeOffset);
    }
    public static Heading createRelativeHeading(float relativeOffset){
        return new Heading(errorCorrecter(relativeOffset + getFieldHeading()));
    }
    public void setRelativeOffset(float relativeOffset){
        this.relativeOffset = relativeOffset;
    }
    public float getRelativeOffset(){
        return relativeOffset;
    }
    public float getRelativeHeading(){
        return errorCorrecter(getFieldHeading() + relativeOffset);
    }
}
