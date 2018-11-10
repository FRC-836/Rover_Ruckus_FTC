package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.WebcamExample;
import org.opencv.core.Mat;

import java.security.Policy;

@TeleOp(name = "Image_Display_Test", group = "main")
public class Image_Display_Test extends OpMode {
//PHONES HAVE TO BE VERTICAL FOR ANY OF THIS TO WORK
    private Gold_Detector detector;
    private final boolean IS_WEBCAM_USED = false;
    private boolean isGoldSeen = false;
    WebcamName webcamName;

    @Override
    public void init() {
        if (IS_WEBCAM_USED) {
            webcamName = hardwareMap.get(WebcamName.class, "w");
        }
        telemetry.addData("Status", "Gold_Test");

        detector = new Gold_Detector();

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0);
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
        public void loop(){
            if(detector.isFound()){
                isGoldSeen = true;
                if(isGoldSeen){
                    double goldXPos = detector.getXPosition();
                    telemetry.addData("X Position", goldXPos);
                    telemetry.addLine("Object Seen");
                    isGoldSeen = false;
                    detector.disable();
            }
            }
            else{
                detector.disable();
            }
        }

        @Override
        public void stop(){
        detector.disable();
        }
    }
//57 to 45



