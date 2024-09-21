package org.firstinspires.ftc.teamcode.Control.Localization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Physical.Hardware;


public class TwoWheelOdometry {

    DcMotor verticalEncoder;
    DcMotor horizontalEncoder;
    Hardware hardware;

    //et variables and measure precision of instruments


    //location variables to be read by other classes
    double x, y, h = 0;

    //what the imu reading should be rounded to for maximum accuracy
    double IMUMaximumPrecision = 0.01;
    double IMUMaxNum;

    //conversion of encoder ticks to cm, and ensures correct direction (pos x pos y like standard coordinate plane)
    public static double ticksToCmHorizontal;
    public static double ticksToCmVertical;

    //defining the center of the robot to be about the center of the vertical encoder
    double horizontalOffset = 15; //vertical distance from horizontal encoder to center of robot
    double verticalOffset = 15; //

    //last vertical and horizontal encoder readings
    double hPrevDist, vPrevDist;

    //new readings
    double newVertical, newHorizontal, newHeading;
    //change in readings / calculated change
    double dVertical, dHorizontal, dHeading, dX, dY;

    //intialize odometry
    public TwoWheelOdometry(double startX, double startY, Hardware hardware){
        x = startX;
        y = startY;
        IMUMaxNum = 1/IMUMaximumPrecision;

        this.hardware = hardware;
        verticalEncoder = hardware.verticalEncoderLeft;
        horizontalEncoder = hardware.horizontalEncoder;

        ticksToCmHorizontal = hardware.getHorizontalEncoderTicksToCm();
        ticksToCmVertical = hardware.getVerticalEncoderTicksToCm();

        //get new reads on sensors (heading, Vertical encoder, and horizontal encoder)
        vPrevDist = verticalEncoder.getCurrentPosition() * ticksToCmVertical;
        hPrevDist = horizontalEncoder.getCurrentPosition() * ticksToCmHorizontal;
    }

    //updating odometry
    public void localize(){
        //get new reads on sensors (heading, Vertical encoder, and horizontal encoder)
        newVertical = verticalEncoder.getCurrentPosition() * ticksToCmVertical;
        newHorizontal = horizontalEncoder.getCurrentPosition() * ticksToCmHorizontal;
        newHeading = hardware.getImuHeading();

        //get change in heading, Vertical encoder, and horizontal encoder
        dVertical = newVertical - vPrevDist;
        dHorizontal = newHorizontal - hPrevDist;
        dHeading = newHeading - h;

        //wraparound issues, does mod pi of angle
        if (dHeading < -Math.PI) { // For example 355 to 5 degrees
            dHeading += 2 * Math.PI;
        } else if (dHeading > Math.PI) { // For example 5 to 355 degrees // IDT NECESSARY
            dHeading -= 2 * Math.PI;
        }

        //Math: https://www.desmos.com/calculator/sfpde8dhcw - incorrect
        //needs more images to be properly explained

        //catch the divide by 0
        if(Math.abs(dHeading) == 0){
            dX = dHorizontal;
            dY = dVertical;
        }else{
            //normal odometry

            double arcRad = (dVertical - (verticalOffset * dHeading)) / dHeading;
            ///  \/ highly debated
            double number = arcRad + dHorizontal - (horizontalOffset * dHeading);

            dX = Math.cos(dHeading) * (number) - arcRad;
            dY = Math.sin(dHeading) * (number);




        }

        //rotating to absolute coordinates vs robot relative calculated above
        x += Math.cos(h) * dX - Math.sin(h) * dY;
        y += Math.sin(h) * dX + Math.cos(h) * dY;

        //update reference values to current position
        vPrevDist = newVertical;
        hPrevDist = newHorizontal;
        h = newHeading;

    }

    public double[] getLocation(){
//        localize();
        return new double[]{x,y,h};
    }

    public Pose2d getPose(){
        return new Pose2d(x, y, new Rotation2d(h));
    }

    public double[] getRawValues(){
        return new double[]{newVertical, newHorizontal, newHeading};
    }

    public void setPosition(double x, double y){
        this.x = x;
        this.y = y;
    }

    public void setPose(Pose2d pose){
        this.x = pose.getX();
        this.y = pose.getY();
    }

}