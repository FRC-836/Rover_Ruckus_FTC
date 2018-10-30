package org.firstinspires.ftc.teamcode;

import com.sun.tools.javac.util.Position;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class Gold_Detector extends Base_Detector {
    private Mat displayMat = new Mat();//will be displayed on the phone
    private Mat mainMat = new Mat();//will worked on the most
    private Mat yellowMat = new Mat();//yellow returned by
    private Mat countorMat = new Mat();

    private boolean found = false;//If we've seen the gold yet
    private boolean aligned = false;
    private double  goldXPos = 0;

    // Detector settings
    public boolean debugAlignment = true;
    public double alignPosOffset = 0;
    public double alignSize = 100;
    public YellowColorFilter yellowColorFilter = new YellowColorFilter();

    public Image_Triangulator triangulator = new Image_Triangulator( 1.0,3);
    public Perfect_Difference_Triangulator perfect_difference_triangulator = new Perfect_Difference_Triangulator(0.01);


    //HSV values for yellow

    public Gold_Detector() {
        super();
        detectorName = "Gold_Detector";
    }


    @Override
    public Mat process(Mat input) {

        input.copyTo(displayMat);
        input.copyTo(mainMat);
        input.release();

        Imgproc.GaussianBlur(mainMat, mainMat, new Size(5, 5), 0);
        yellowColorFilter.process(mainMat.clone(), yellowMat);

        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Imgproc.findContours(yellowMat, contoursYellow, countorMat, Imgproc.RETR_TREE, Imgproc.
                CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat, contoursYellow, -1, new Scalar(230, 70, 70),
                2);

        Rect bestRect = null;
        double bestDifference = Double.MAX_VALUE;

        for (MatOfPoint cont : contoursYellow) {
            double score = calculateScore(cont); // Get the diffrence score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0, 0, 255), 2); // Draw rect

            // If the result is better then the previously tracked one, set this rect as the new best
            if (score < bestDifference) {
                bestDifference = score;
                bestRect = rect;
            }

        }
        double alignX    = (getAdjustedSize().width / 2) + alignPosOffset; // Center point in X Pixels
        double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
        double alignXMax = alignX +(alignSize / 2); // Max X pos in pixels
        double xPos;

        if (bestRect != null) {
            Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(255, 0, 0), 4);
            Imgproc.putText(displayMat, "Chosen", bestRect.tl(), 0, 1, new Scalar(255, 255, 255));

            xPos = bestRect.x + (bestRect.width / 2);
            goldXPos = xPos;

            Imgproc.circle(displayMat, new Point( xPos, bestRect.y + (bestRect.height / 2)), 5, new Scalar(0,255,0),2);

            if(xPos < alignXMax && xPos > alignXMin){
                aligned = true;
            }else{
                aligned = false;
            }
            Imgproc.putText(displayMat,"Current X: " + bestRect.x,new Point(10,getAdjustedSize().height - 10),0,0.5, new Scalar(255,255,255),1);
            found = true;

        } else {
            found = false;
            aligned = false;
        }
        if(debugAlignment){

            //Draw debug alignment info
            if(isFound()){
                Imgproc.line(displayMat,new Point(goldXPos, getAdjustedSize().height), new Point(goldXPos, getAdjustedSize().height - 30),new Scalar(255,255,0), 2);
            }

            Imgproc.line(displayMat,new Point(alignXMin, getAdjustedSize().height), new Point(alignXMin, getAdjustedSize().height - 40),new Scalar(0,255,0), 2);
            Imgproc.line(displayMat,new Point(alignXMax, getAdjustedSize().height), new Point(alignXMax,getAdjustedSize().height - 40),new Scalar(0,255,0), 2);
        }
        Imgproc.putText(displayMat,"Result: " + aligned,new Point(10,getAdjustedSize().height - 30),0,1, new Scalar(255,255,0),1);


        return displayMat;

    }
    @Override
    public void useDefaults(){
        addScorer(triangulator);
        addScorer(perfect_difference_triangulator);

    }
    public boolean getAligned(){
        return aligned;
    }


    public double getXPosition(){
        return goldXPos;
    }



    public boolean isFound() {
        return found;
    }
}


