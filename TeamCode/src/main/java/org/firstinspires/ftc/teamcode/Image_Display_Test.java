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
    public boolean isRight = false;
    public boolean isLeft = false;
    public boolean isCenter = false;
    public boolean debug = true;

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
        public void loop() {
            if (debug != true) {
                if (detector.isFound()) {
                    isGoldSeen = true;
                    double goldXPos = detector.getXPosition();
                    telemetry.addData("X Position", goldXPos);
                    telemetry.addLine("Object Seen");
                    goldPosDetector();
                    isGoldSeen = false;
                    detector.disable();
                }
                else{
                    detector.disable();
                }
            } else {
                double goldXPos = detector.getXPosition();
                telemetry.addData("X Position", goldXPos);
                goldPosDetector();
            }

        }
        @Override
        public void stop(){
        detector.disable();
        }
        public void goldPosDetector(){
        //change inequalities depending on camera placement of robot-change this as the camera mount changes
            double goldXPos = detector.getXPosition();
            if(0 <= goldXPos && 15 >= goldXPos){
                isLeft = true;
                telemetry.addLine("Gold Position: Left");

            }
            else if(40 <= goldXPos && 60 >= goldXPos){
                isCenter = true;
                telemetry.addLine("Gold Position: Center");
            }
            else if(95 <= goldXPos && 120 >= goldXPos){
                isRight = true;
                telemetry.addLine("Gold Position: Right");
            }
        }
    }
//57 to 45



