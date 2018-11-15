package org.firstinspires.ftc.teamcode;

import android.graphics.Color;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Example", group = "Sensor")
public class ColorSensor_RGB_HSV extends LinearOpMode {

   private ColorSensor colorSensor;

    private float[] hsvValues = {0f, 0f, 0f};

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        waitForStart();

        while (opModeIsActive()) {
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            Color.RGBToHSV(red, green, blue, hsvValues);

            telemetry.addData("Red  ", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue ", blue);
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Saturation", hsvValues[1]);
            telemetry.addData("Value", hsvValues[2]);
        }
    }
}