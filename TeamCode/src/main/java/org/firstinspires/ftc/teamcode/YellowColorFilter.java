package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class YellowColorFilter extends Base_Color_Filter {
    private Imgproc image;
    private Imgproc implementedImage;
    private Mat hsv = new Mat();
    private double threshold = -1;
    private List<Mat> channels = new ArrayList<>();

    public YellowColorFilter(){
        updateSettings( -1);
    }


    public YellowColorFilter(double filterThreshold){
        updateSettings(filterThreshold);
    }

    private void updateSettings(double filterThreshold){
        threshold = filterThreshold;
    }
    @Override
    public void process(Mat input, Mat mask) {
        channels = new ArrayList<>();
        if (threshold == -1) {
            threshold = 70;
        }

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV);
        Core.split(input, channels);
        if (channels.size() > 0) {
            Imgproc.threshold(channels.get(1), mask, threshold, 255, Imgproc.THRESH_BINARY_INV);
        }
        for (int i = 0; i < channels.size(); i++) {
            channels.get(i).release();
        }

        input.release();
    }
}

