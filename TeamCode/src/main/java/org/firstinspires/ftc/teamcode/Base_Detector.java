package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
public abstract class Base_Detector extends OpenCVPipeline {

    public abstract Mat process(Mat input);
    public abstract void useDefaults();


    private List<Base_Image_Triangulator> scorers = new ArrayList<>();
    private Size firstSize;
    private Size adjustedSize;
    private Mat mainMat = new Mat();

    public Speed_Settings.detectionSpeed speed = Speed_Settings.detectionSpeed.MODERATE;
    public double reducedImageRatio = 0.5; //This'll have to be tested for sure
    public Size reducedImageQuality = new Size(640, 480);//just a guess
    protected String detectorName = "Base_Detector";
    public double maxDifference = 10;


    public Base_Detector(){
        //this'll be our class constructor
    }


    public void setSpeed(Speed_Settings.detectionSpeed speed){
        this.speed = speed;
    }

    public double calculateScore(Mat input){
        double totalScore = 0;
        for(Base_Image_Triangulator scorer : scorers){
            totalScore += scorer.calculateScore(input);
        }

        return totalScore;
    }

    public void addScorer(Base_Image_Triangulator newScorer){
        scorers.add(newScorer);
    }

    public Mat processFrame(Mat rgba, Mat gray) {
        firstSize = rgba.size();
        adjustedSize = new Size(firstSize.width * reducedImageRatio, firstSize.height
                * reducedImageRatio);

        rgba.copyTo(mainMat);

        if (rgba.empty()){return rgba; }

        Imgproc.resize(mainMat, mainMat, adjustedSize);//thou shalt be shrunken
        Imgproc.resize(process(mainMat), mainMat, getFirstSize());//thou shalt be made big again
        Imgproc.putText(mainMat,"389 Computer Vision" + detectorName + ": " +
                getAdjustedSize().toString() + " - " + speed.toString(), new Point(5,30),0,0.5,
                new Scalar(0,255,255),2);

        return mainMat;
    }
    // some get methods for sizes-we'll use them-not sure how to use them tho
    public Size getFirstSize() {
        return firstSize;
    }
    public Size getAdjustedSize() {
        return adjustedSize;
    }
    public void setAdjustedSize(Size size) {
        this.adjustedSize = size;
    }
}


