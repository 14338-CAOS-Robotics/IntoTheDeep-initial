package org.firstinspires.ftc.teamcode.Vision.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class PezDetector extends OpenCvPipeline {
    Mat yellowMask = new Mat();
    Mat blueMask = new Mat();
    Mat redMask = new Mat();
    Mat hsv = new Mat();
    public Scalar yellowLow = new Scalar(75.1,86.4,0,0);
    public Scalar yellowHigh = new Scalar(255,177.1,53.8,255);
    public Scalar redLow = new Scalar(100,0,0,0);
    public Scalar redHigh = new Scalar(255,62,48.1,255);
    public Scalar blueLow = new Scalar(0,0,85,0);
    public Scalar blueHigh = new Scalar(70,86,179,255);
    @Override
    public Mat processFrame(Mat input) {
        hsv = new Mat();

        //Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(input, yellowLow, yellowHigh, yellowMask);
        Core.inRange(input, redLow, redHigh, redMask);
        Core.inRange(input, blueLow, blueHigh, blueMask);

        Core.bitwise_or(yellowMask, redMask, yellowMask);
        Core.bitwise_or(yellowMask, blueMask, yellowMask);

        Core.bitwise_and(input, input, hsv, yellowMask);
        return hsv;
    }
}
