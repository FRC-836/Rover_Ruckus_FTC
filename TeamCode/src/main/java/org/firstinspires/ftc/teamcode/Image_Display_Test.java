package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.Mat;

@TeleOp(name = "Image_Display_Test", group = "main")
public class Image_Display_Test extends OpMode {
    private Gold_Detector detector;


    @Override
    public void init() {
        telemetry.addData("Status", "Gold_Test");

        detector = new Gold_Detector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.alignSize = 100;
        detector.alignPosOffset = 0;
        detector.reducedImageRatio = 0.4;

        detector.triangulator.perfectRatio = 1.0;
        detector.triangulator.saturation = 0.005;
        detector.perfect_difference_triangulator.weight = 0.005;

        detector.enable();


    }

    @Override
    public void init_loop() {
    }


    @Override
    public void start() {

    }


    @Override
    public void loop() {
        telemetry.addData("IsAligned" , detector.getAligned());
        telemetry.addData("X Pos" , detector.getXPosition());
    }

    @Override
    public void stop() {
        detector.disable();
    }

}

