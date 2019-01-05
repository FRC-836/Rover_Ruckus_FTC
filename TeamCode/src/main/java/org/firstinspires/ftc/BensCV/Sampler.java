package org.firstinspires.ftc.BensCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Sampler {
    private Gold_Detector detector;
    private boolean debug = true;//this is the only part of this program that should be changed
    private boolean isGoldSeen = false;
    private boolean isLeft = false;
    private boolean isCenter = false;
    private boolean isRight = false;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public enum position{
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    public Sampler(HardwareMap hardwareMap, Telemetry telemetry, boolean debug){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.debug = debug;
    }
    
    public void initialize(){
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
    public void run(){
        if (!debug) {
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
    public void halt(){
        detector.disable();
    }
    public position  goldPosDetector() {
            double goldXPos = detector.getXPosition();
            if (0.0 <= goldXPos && 214.0 >= goldXPos) {
                isLeft = true;
                telemetry.addLine("Gold Position: Left");
                return position.LEFT;
            } else if (214.0 < goldXPos && 428.0 >= goldXPos) {
                isCenter = true;
                telemetry.addLine("Gold Position: Center");
                return position.CENTER;

            } else if (428.0 < goldXPos && 640.0 >= goldXPos) {
                isRight = true;
                telemetry.addLine("Gold Position: Right");
                return position.RIGHT;
            } else {
                return position.NONE;
            }
        }
    }
