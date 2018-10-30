package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;

public class Perfect_Difference_Triangulator extends Base_Image_Triangulator {
    public double weight       = 1.0;
    public Perfect_Difference_Triangulator( double weight){
        this.weight = weight;

    }

    @Override
    public double calculateScore(Mat input) {
        if(!(input instanceof MatOfPoint)) return Double.MAX_VALUE;
        MatOfPoint contour = (MatOfPoint) input;
        double area = Imgproc.contourArea(contour);

        return -area * weight;
    }
}
