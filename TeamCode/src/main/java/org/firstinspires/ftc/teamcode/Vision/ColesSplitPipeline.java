package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
@Config
public class ColesSplitPipeline extends OpenCvPipeline{

    public static int lowUpper = 255;
    public static int highUpper = 255;
    public static int luminosityHigh = 255;
    public static int luminosityLow = 0;
    public static int lowBlue = 160;
    public static int lowRed = 0;

    public static double horizon = 600;
    public static double minArea = 1000;

    private int regionIndex = 0;


    private Mat YCrCb = new Mat();
    private Mat binaryMat = new Mat();
    private Mat maskedInputMat = new Mat();
    private Mat contoursOnPlainImageMat = new Mat();

    private ArrayList<MatOfPoint> contoursList = new ArrayList<>();
    private ArrayList<MatOfPoint> filteredContours = new ArrayList<>();

    private ArrayList<RotatedRect> rects = new ArrayList<>();

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0,255,0);
    static final Scalar[] COLORS = {BLUE, RED, GREEN};
    static final int CONTOUR_LINE_THICKNESS = 2;

    static final int[] RED_REGIONS = {1,2,3};
    static final int[] BLUE_REGIONS = {1,2,3};
    boolean isRed;
    boolean detected;

    public ColesSplitPipeline(boolean isRed) {
         this.isRed=isRed;
         detected = false;

    }

    public boolean hasProcessed() {
        return detected;
    }



    static void drawRotatedRect(RotatedRect rect, Mat drawOn, Scalar color)
    {
        /*
         * Draws a rotated rect by drawing each of the 4 lines individually
         */

        Point[] points = new Point[4];
        rect.points(points);

        for(int i = 0; i < 4; ++i)
        {
            Imgproc.line(drawOn, points[i], points[(i+1)%4], color, 2);
        }
    }


    static Point getCenter(MatOfPoint cnt) {
        Moments m = Imgproc.moments(cnt, false);
        double x = m.get_m10() / m.get_m00();
        double y = m.get_m01() / m.get_m00();
        Point center = new Point(x,y);
        return center;
    }

    /*static double getFullness(MatOfPoint cnt, RotatedRect rect) {
        return Imgproc.contourArea(cnt) / rect.size.area();

    }*/

    public int getTargetPosition() {

        int[] targets;
        if (isRed) targets = RED_REGIONS; else targets = BLUE_REGIONS;
        return targets[regionIndex];

    }

    @Override
    public Mat processFrame(Mat input) {


        Scalar lower = new Scalar(luminosityLow, lowBlue, lowRed);
        Scalar upper;
        if (!isRed) { upper = new Scalar(luminosityHigh, lowUpper, highUpper);
            lower = new Scalar(luminosityLow, lowRed, lowBlue);}
        else upper = new Scalar(luminosityHigh, highUpper, lowUpper);
        Comparator<RotatedRect> rectSortX = (r1,r2) -> (int) (r1.center.x - r2.center.x);
        Comparator<MatOfPoint> contourSortX = (c1, c2) -> (int) (getCenter(c1).x - getCenter(c2).x);
        contoursList.clear();
        filteredContours.clear();
        rects.clear();
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);


        //0 = not seen, 1 = left, 2 = right



        double centerLine = input.width()/2;
        Point topCenter = new Point(4*centerLine, 0);
        Point bottomCenter = new Point(4*centerLine, 4*input.height());

        Point leftHorizon = new Point(0,horizon);
        Point rightHorizon = new Point(input.width()*4, horizon);

        Core.inRange(YCrCb, lower, upper, binaryMat);
        maskedInputMat.release();
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);
        Imgproc.findContours(binaryMat, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        for (MatOfPoint cnt : contoursList) {
            if ((Imgproc.contourArea(cnt, false) > minArea) && getCenter(cnt).y > horizon/4) {
                filteredContours.add(cnt);
            }
        }
        maskedInputMat.copyTo(contoursOnPlainImageMat);
        Imgproc.drawContours(contoursOnPlainImageMat, filteredContours, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);
        double maxArea = 0;
        for (int i = 0; i < filteredContours.size(); i++) {
            MatOfPoint contour = filteredContours.get(i);
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            if (rect.size.area() > maxArea) maxArea = rect.size.area();
            //double fullness =  getFullness(contour, rect);
            //if (fullness < leastFullness) leastFullness = fullness;
            //drawRotatedRect(rect, contoursOnPlainImageMat, RED);
            rects.add(i, rect);

        }
        rects.sort(rectSortX);
        filteredContours.sort(contourSortX);

        Imgproc.line(input, topCenter, bottomCenter, BLUE, CONTOUR_LINE_THICKNESS, CONTOUR_LINE_THICKNESS, CONTOUR_LINE_THICKNESS);
        Imgproc.line(input, leftHorizon, rightHorizon, GREEN, CONTOUR_LINE_THICKNESS, CONTOUR_LINE_THICKNESS, CONTOUR_LINE_THICKNESS);


        if (maxArea == 0) regionIndex = 0;
        for (int i=0; i < rects.size(); i++) {
            RotatedRect rect = rects.get(i);
            //MatOfPoint cnt = filteredContours.get(i);

            if (rect.size.area() == maxArea) {
                //Yippee! I've found it
                if (rect.center.x < centerLine) {
                    regionIndex = 1;
                    drawRotatedRect(rect, input, RED);
                } else {
                    regionIndex = 2;
                    drawRotatedRect(rect, input, GREEN);
                }

            }

            /*if (getFullness(cnt, rect) == leastFullness) {
                //You did it! You found the team element
                drawRotatedRect(rect, input, RED);

            }
            else{
                drawRotatedRect(rect, contoursOnPlainImageMat, GREEN);
            }*/

            //drawRotatedRect(rect, contoursOnPlainImageMat, RED);
            //Imgproc.putText(contoursOnPlainImageMat, String.valueOf(getFullness(cnt, rect)), getCenter(cnt), 2, 0.5, GREEN);
        }
        detected = true;
        return input;
    }

}
