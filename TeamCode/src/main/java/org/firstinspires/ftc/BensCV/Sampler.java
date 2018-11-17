package org.firstinspires.ftc.BensCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Sampler {
    private Gold_Detector detector;
    private boolean isWebcamUsed;
    private boolean isVertical;
    private WebcamName webcamName;
    private boolean debug = true;//this is the only part of this program that should be changed
    private boolean isGoldSeen = false;
    private boolean isLeft = false;
    private boolean isCenter = false;
    private boolean isRight = false;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public position positionIdentifier = position.NONE;

    public enum position {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    public Sampler(boolean isWebcamUsed, boolean isVertical, HardwareMap hardwareMap, Telemetry telemetry, boolean debug){
        this.isWebcamUsed = isWebcamUsed;
        this.isVertical = isVertical;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.debug = debug;
    }
    
    public void initialize(){
        if (isWebcamUsed) {
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
    public void run(){
        if (!debug) {
            if (detector.isFound()) {
                isGoldSeen = true;
                double goldXPos = detector.getXPosition();
                double goldYPos = detector.getYPosition();
                telemetry.addData("X Position", goldXPos);
                telemetry.addData("Y Position", goldYPos);
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
            double goldYPos = detector.getYPosition();
            telemetry.addData("X Position", goldXPos);
            telemetry.addData("Y Position", goldYPos);
            goldPosDetector();
        }
    }
    public void halt(){
        detector.disable();
    }
    public position goldPosDetector() {
        //change inequalities depending on camera placement of robot-change this as the camera mount changes
        if (isVertical) {
            double goldXPos = detector.getXPosition();
            if (0.0 <= goldXPos && 60.0 >= goldXPos) {
                isLeft = true;
                telemetry.addLine("Gold Position: Left");
                positionIdentifier = position.LEFT;
            } else if (60.0 < goldXPos && 120.0 >= goldXPos) {
                isCenter = true;
                telemetry.addLine("Gold Position: Center");
                positionIdentifier = position.CENTER;

            } else if (120.0 < goldXPos && 180.0 >= goldXPos) {
                isRight = true;
                telemetry.addLine("Gold Position: Right");
                positionIdentifier = position.RIGHT;
            } else {
                positionIdentifier = position.NONE;
            }
        }
        else {
            double goldYPos = detector.getYPosition();
            if (0.0 <= goldYPos && 80.0 >= goldYPos) {
                isLeft = true;
                telemetry.addLine("Gold Position: Right");
                positionIdentifier = position.RIGHT;
            } else if (80.0 < goldYPos && 160.0 >= goldYPos) {
                isCenter = true;
                telemetry.addLine("Gold Position: Center");
                positionIdentifier = position.CENTER;

            } else if (160.0 < goldYPos && 240.0 >= goldYPos) {
                isRight = true;
                telemetry.addLine("Gold Position: Left");
                positionIdentifier = position.LEFT;
            } else {
                positionIdentifier = position.NONE;
            }
        }
        return positionIdentifier;
    }


}
