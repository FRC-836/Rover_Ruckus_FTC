package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class Image_Triangulator extends Base_Image_Triangulator {
    public double perfectRatio = 1.0;
    public double saturation = 1.0;
    public Image_Triangulator(){}

    public Image_Triangulator(double perfectRatio, double saturation){
        this.saturation = saturation;
        this.perfectRatio = perfectRatio;
    }
    @Override
    public double calculateScore(Mat input){
        if (!(input instanceof MatOfPoint)) return Double.MAX_VALUE;
        MatOfPoint contor = (MatOfPoint) input;

        Rect rect = Imgproc.boundingRect(contor);
        double upLength = rect.x;
        double upWidth = rect.y;
        double downLength = rect.width;
        double downWidth = rect.height;

        double cubeRatio  = Math.max(Math.abs(downLength/downWidth), Math.abs(downWidth/downLength));
        double ratioDifference = Math.abs(cubeRatio - perfectRatio);
        return ratioDifference * saturation;
    }

}
