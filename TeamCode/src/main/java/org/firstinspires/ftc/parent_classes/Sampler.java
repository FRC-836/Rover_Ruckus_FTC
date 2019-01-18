package org.firstinspires.ftc.parent_classes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Vector;

public class Sampler {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AVOMBtn/////AAAAGWoe5Xr26kwMi197rJ9ukz9CynZI0gtPtGmyzJJF/DX5fVRX6KZpGATlLq8PX8m1UBpwePrOZAkZmI/XhtlNcZ3fbgvlGuYfDwkaUgbaJgrzPut88uA5KdcubkI0uTw/J+S2y/jGLeKM3pnXMxywmGjbNNGRLgt57I9pThzUjuwC8jNi42C3Qo17qpTiHOFRWbRybxGDKk2PAUEVdCFtjW4zuzB+b5xBABGCytzwdPk+riWGBSITihc1tPIWtOP1CDQiMp4B3V1ysJTPxOTXkWmFuSqqFhgePWKoXxVqbcWVcpUDQ1yVPdpK9BxrZfeoVrpWGRfBa9mrSYsB6VBKkKIxMClow9gxYv5SyN440Btn";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private Telemetry myTelemetry;

    private final float LEFT_OF_SCREEN = 427.0f;
    private final float RIGHT_OF_SCREEN = 853.0f;

    private final float VERTICAL_CUTOFF = 10000.0f;

    public enum GoldPosition {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.myTelemetry = telemetry;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null)
            tfod.activate();
    }

    private float getRecVerticalPos(Recognition recognition) {
        float vertPos = (recognition.getBottom() + recognition.getTop()) / 2.0f;
        myTelemetry.addData("Vertical Pos",vertPos);
        return vertPos;
    }

    private float getRecHorizontalPos(Recognition recognition) {
        float horzPos = (recognition.getLeft() + recognition.getRight()) / 2.0f;
        myTelemetry.addData("Side Pos",horzPos);
        return horzPos;
    }

    public GoldPosition sample() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                // Code to react to updated recognitions goes below this line!
                Vector<Recognition> silverList = new Vector<>();
                Recognition bestGold = null;
                for (Recognition newRecognition : updatedRecognitions) {
                    if (getRecVerticalPos(newRecognition) > VERTICAL_CUTOFF)
                        continue;
                    if (newRecognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        if (bestGold == null)
                            bestGold = newRecognition;
                        else
                            bestGold = findBestGold(newRecognition, bestGold);
                    } else if (newRecognition.getLabel().equals(LABEL_SILVER_MINERAL)){
                        silverList.add(newRecognition);
                    }
                }
                if (silverList.size() > 2)
                    silverList = findBestTwoSilver(silverList);


                if (bestGold == null)
                {
                    if (silverList.size() == 2)
                    {
                        float silverPos1 = getRecVerticalPos(silverList.elementAt(0));
                        float silverPos2 = getRecVerticalPos(silverList.elementAt(1));
                        if ((silverPos1 < LEFT_OF_SCREEN && silverPos2 < LEFT_OF_SCREEN) ||
                                (silverPos1 > RIGHT_OF_SCREEN && silverPos2 > RIGHT_OF_SCREEN) ||
                                (silverPos1 >= LEFT_OF_SCREEN && silverPos2 >= LEFT_OF_SCREEN &&
                                silverPos1 <= RIGHT_OF_SCREEN && silverPos2 <= RIGHT_OF_SCREEN))
                        {
                            silverList.remove(1);
                        }
                        else
                        {
                            boolean silverLeft = silverPos1 < LEFT_OF_SCREEN || silverPos2 < LEFT_OF_SCREEN;
                            boolean silverRight = silverPos1 > RIGHT_OF_SCREEN || silverPos2 > RIGHT_OF_SCREEN;
                            if (!silverLeft)
                                return GoldPosition.LEFT;
                            if (!silverRight)
                                return GoldPosition.RIGHT;
                            return GoldPosition.CENTER;
                        }
                    }
                    if (silverList.size() == 1)
                    {
                        float silverPos = getRecVerticalPos(silverList.elementAt(0));
                        if (silverPos <= LEFT_OF_SCREEN)
                            return GoldPosition.CENTER;
                        else if (silverPos < RIGHT_OF_SCREEN)
                            return GoldPosition.RIGHT;
                        else
                            return GoldPosition.CENTER;
                    }
                    if (silverList.size() == 0)
                    {
                        return GoldPosition.UNKNOWN;
                    }
                }
                else
                {
                    switch (silverList.size())
                    {
                        case 0:
                        case 1:
                        case 2:
                            // Use the gold position alone to determine location
                            if (getRecVerticalPos(bestGold) > RIGHT_OF_SCREEN)
                                return GoldPosition.RIGHT;
                            else if (getRecVerticalPos(bestGold) < LEFT_OF_SCREEN)
                                return GoldPosition.LEFT;
                            else
                                return GoldPosition.CENTER;
                            /*
                            // Determine location of gold based on location relative to 2 silvers
                            if (getRecVerticalPos(bestGold) > getRecVerticalPos(silverList.elementAt(0)) &&
                                    getRecVerticalPos(bestGold) > getRecVerticalPos(silverList.elementAt(1)))
                                return GoldPosition.LEFT;
                            else if (getRecVerticalPos(bestGold) < getRecVerticalPos(silverList.elementAt(0)) &&
                                    getRecVerticalPos(bestGold) < getRecVerticalPos(silverList.elementAt(1)))
                                return GoldPosition.RIGHT;
                            else
                                return GoldPosition.CENTER;
                                */
                    }
                }
                // Code to react to updated recognitions goes above this line!
            }
        }
        myTelemetry.update();
        return GoldPosition.UNKNOWN;
    }

    private Vector<Recognition> findBestTwoSilver(Vector<Recognition> silverList) {
        // TODO: Make more efficient
        Vector<Recognition> finalList = new Vector<>();
        int bestI = 0;
        float bestPos = getRecHorizontalPos(silverList.elementAt(bestI));
        for (int i = 1; i < silverList.size(); i++)
        {
            float pos = getRecHorizontalPos(silverList.elementAt(i));
            if (pos < bestPos)
            {
                bestI = i;
                bestPos = pos;
            }
        }
        finalList.add(silverList.elementAt(bestI));

        int secondBestI = 0;
        if (bestI == 0) secondBestI = 1;
        float secondBestPos = getRecHorizontalPos(silverList.elementAt(secondBestI));
        for (int i = secondBestI + 1; i < silverList.size(); i++)
        {
            float pos = getRecHorizontalPos(silverList.elementAt(i));
            if (pos < secondBestPos && i != bestI)
            {
                secondBestI = i;
                secondBestPos = pos;
            }
        }
        finalList.add(silverList.elementAt(secondBestI));

        return finalList;
    }

    private Recognition findBestGold(Recognition rec1, Recognition rec2) {
        //if (rec1.getConfidence() > rec2.getConfidence())
        if (getRecHorizontalPos(rec1) < getRecHorizontalPos(rec2))
            return rec1;
        else
            return rec2;
    }

    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}