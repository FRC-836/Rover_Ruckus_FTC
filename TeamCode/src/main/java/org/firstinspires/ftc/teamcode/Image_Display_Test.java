package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.BensCV.Gold_Detector;
import org.firstinspires.ftc.BensCV.Sampler;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name = "Image_Display_Test", group = "main")
public class Image_Display_Test extends OpMode {
    private Sampler sampler;

    @Override
    public void init() {
        sampler = new Sampler(false,false, hardwareMap, telemetry);
        sampler.initialize();
    }

    @Override
    public void loop() {
        sampler.run();
    }
    @Override
    public void stop(){
        sampler.halt();
    }
}
//57 to 45



