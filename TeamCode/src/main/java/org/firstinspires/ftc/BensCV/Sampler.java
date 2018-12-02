package org.firstinspires.ftc.BensCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Sampler {
    private Gold_Detector detector;
    private boolean isVertical;
    private boolean debug = true;//this is the only part of this program that should be changed
    private boolean isGoldSeen = false;
    private boolean isLeft = false;
    private boolean isCenter = false;
    private boolean isRight = false;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private position positionIdentifier = position.NONE;

    public enum position {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }
    public Sampler( boolean isVertical, HardwareMap hardwareMap, Telemetry telemetry, boolean debug){
        this.isVertical = isVertical;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.debug = debug;
    }
    
    public void initialize(){

        telemetry.addData("Status", "Gold_Test");

        detector = new Gold_Detector();

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), -1);//was 0
        detector.useDefaults();
        detector.alignSize = 100;
        detector.alignPosOffset = 0;
        detector.reducedImageRatio = 0.4;

        detector.triangulator.perfectRatio = 1.0;
        detector.triangulator.saturation = 0.005;
        detector.perfect_difference_triangulator.weight = 0.005;

        detector.enable();
    }
    public position run(){
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
        return positionIdentifier;
    }
    public void halt(){
        detector.disable();
    }
    public void goldPosDetector() {
        if (isVertical) {
            double goldXPos = detector.getXPosition();
            if (0.0 <= goldXPos && 20.0 >= goldXPos) {
                isLeft = true;
                telemetry.addLine("Gold Position: Right");
                positionIdentifier = position.LEFT;
            } else if (20.0 < goldXPos && 360.0 >= goldXPos) {
                isCenter = true;
                telemetry.addLine("Gold Position: Center");
                positionIdentifier = position.CENTER;
            } else if (360.0 < goldXPos && 500.0 >= goldXPos) {
                isRight = true;
                telemetry.addLine("Gold Position: Left");
                positionIdentifier = position.RIGHT;
            } else {
                positionIdentifier = position.NONE;
            }
        }
        else {
            double goldYPos = detector.getXPosition();
            if (0.0 <= goldYPos && 20.0 >= goldYPos) {
                isLeft = true;
                telemetry.addLine("Gold Position: Left");
                positionIdentifier = position.LEFT;
            } else if (20.0 < goldYPos && 400.0 >= goldYPos) {
                isCenter = true;
                telemetry.addLine("Gold Position: Center");
                positionIdentifier = position.CENTER;

            } else if (400.0 < goldYPos && 580.0 >= goldYPos) {
                isRight = true;
                telemetry.addLine("Gold Position: Right");
                positionIdentifier = position.RIGHT;
            } else {
                positionIdentifier = position.NONE;
            }
        }

    }


}
