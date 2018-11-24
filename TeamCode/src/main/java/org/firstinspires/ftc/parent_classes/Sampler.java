package org.firstinspires.ftc.parent_classes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

public class Sampler {
    private static final String VUFORIA_KEY = "AVOMBtn/////AAAAGWoe5Xr26kwMi197rJ9ukz9CynZI0gtPtGmyzJJF/DX5fVRX6KZpGATlLq8PX8m1UBpwePrOZAkZmI/XhtlNcZ3fbgvlGuYfDwkaUgbaJgrzPut88uA5KdcubkI0uTw/J+S2y/jGLeKM3pnXMxywmGjbNNGRLgt57I9pThzUjuwC8jNi42C3Qo17qpTiHOFRWbRybxGDKk2PAUEVdCFtjW4zuzB+b5xBABGCytzwdPk+riWGBSITihc1tPIWtOP1CDQiMp4B3V1ysJTPxOTXkWmFuSqqFhgePWKoXxVqbcWVcpUDQ1yVPdpK9BxrZfeoVrpWGRfBa9mrSYsB6VBKkKIxMClow9gxYv5SyN440Btn";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public enum GoldPosition {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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

    public GoldPosition sample() {
        if (tfod != null) {
            /* getUpdatedRecognitions() will return null if no new information is available since
             the last time that call was made. */
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 3) {
                    int goldMineralY = -1;
                    int silverMineral1Y = -1;
                    int silverMineral2Y = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralY = (int) recognition.getBottom();
                        } else if (silverMineral1Y == -1) {
                            silverMineral1Y = (int) recognition.getBottom();
                        } else {
                            silverMineral2Y = (int) recognition.getBottom();
                        }
                    }
                    if (goldMineralY != -1 && silverMineral1Y != -1 && silverMineral2Y != -1) {
                        if (goldMineralY < silverMineral1Y && goldMineralY < silverMineral2Y) {
                            return GoldPosition.LEFT;
                        } else if (goldMineralY > silverMineral1Y && goldMineralY > silverMineral2Y) {
                            return GoldPosition.RIGHT;
                        } else {
                            return GoldPosition.CENTER;
                        }
                    }
                }
            }
        }
        return GoldPosition.UNKNOWN;
    }

    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}