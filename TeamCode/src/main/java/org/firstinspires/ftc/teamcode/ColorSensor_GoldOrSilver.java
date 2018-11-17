package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "Color Sensor Test")
public class Color_Sensor_Test extends OpMode {

    final double SCALE_FACTOR = 255;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    float hsvValues[] = {0F, 0F, 0F};


    @Override
    public void init() {
        sensorColor = hardwareMap.get(ColorSensor.class, "Sense Color/Dist");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "Sense Color/Dist");
    }


    @Override
    public void loop() {
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        telemetry.addData("Distance (In)", sensorDistance.getDistance(DistanceUnit.INCH));

        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);

        if (hsvValues[1] > 0.45)
        {
            // Gold
            telemetry.addLine("Gold");
        }
        else
        {
            // Silver
            telemetry.addLine( "Silver");
        }

        telemetry.update();

    }
}
