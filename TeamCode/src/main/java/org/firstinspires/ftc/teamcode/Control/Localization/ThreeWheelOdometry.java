package org.firstinspires.ftc.teamcode.Control.Localization;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Physical.Hardware;

@Config
public class ThreeWheelOdometry {

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.69; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double REAL_LATERAL_DISTANCE = 9.43015748;// in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -0.1864566929;
    double encoderMultiplier = WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO / TICKS_PER_REV;
    DcMotor verticalLeftEncoder, verticalRightEncoder;
    DcMotor horizontalEncoder;
    Hardware hardware;

    public double LATERAL_DISTANCE = REAL_LATERAL_DISTANCE;
    //et variables and measure precision of instruments


    //location variables to be read by other classes
    double x, y, h = 0;

    //what the imu reading should be rounded to for maximum accuracy


    //conversion of encoder ticks to cm, and ensures correct direction (pos x pos y like standard coordinate plane)
    public static double ticksToCmHorizontal;
    public static double ticksToCmVertical;

    //defining the center of the robot to be about the center of the vertical encoder
    double horizontalOffset = 15; //vertical distance from horizontal encoder to center of robot
    double verticalOffset = 15; //

    //last vertical and horizontal encoder readings
    double hPrevDist, vlPrevDist, vrPrevDist;

    //new readings
    double newVerticalLeft, newVerticalRight, newHorizontal, newHeading;
    //change in readings / calculated change
    double dVerticalLeft, dVerticalRight, dHorizontal, dHeading, dX, dY;

    //intialize odometry
    public ThreeWheelOdometry(double startX, double startY, double startRot, Hardware hardware){
        x = startX;
        y = startY;
        h = startRot;


        this.hardware = hardware;
        verticalLeftEncoder = hardware.verticalEncoderLeft;
        verticalRightEncoder = hardware.verticalEncoderRight;
        horizontalEncoder = hardware.horizontalEncoder;

        ticksToCmHorizontal = hardware.getHorizontalEncoderTicksToCm();
        ticksToCmVertical = hardware.getVerticalEncoderTicksToCm();

        //get new reads on sensors (heading, Vertical encoder, and horizontal encoder)
        vlPrevDist = verticalLeftEncoder.getCurrentPosition() * encoderMultiplier;
        vrPrevDist = verticalRightEncoder.getCurrentPosition() * encoderMultiplier;
        hPrevDist = horizontalEncoder.getCurrentPosition() * encoderMultiplier;
    }

    //updating odometry
    public void localize(){
        //get new reads on sensors (heading, Vertical encoder, and horizontal encoder)
        newVerticalLeft = verticalLeftEncoder.getCurrentPosition() * encoderMultiplier;
        newVerticalRight = verticalRightEncoder.getCurrentPosition() * encoderMultiplier;
        newHorizontal = horizontalEncoder.getCurrentPosition() * encoderMultiplier;


        //get change in heading, Vertical encoder, and horizontal encoder
        dVerticalLeft = newVerticalLeft - vlPrevDist;
        dVerticalRight = newVerticalRight - vrPrevDist;
        dHorizontal = newHorizontal - hPrevDist;
        dHeading = (dVerticalLeft - dVerticalRight) / LATERAL_DISTANCE;

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
            dY = dVerticalLeft;
        }else{

            dX = (dVerticalLeft + dVerticalRight) / 2;
            dY = ((FORWARD_OFFSET / LATERAL_DISTANCE)*(dVerticalRight - dVerticalLeft) + dHorizontal);
//            dHeading = (dVerticalRight - dVerticalLeft) / LATERAL_DISTANCE;
            //normal odometry

//            double arcRad = (dVertical - (verticalOffset * dHeading)) / dHeading;
//            ///  \/ highly debated
//            double number = arcRad + dHorizontal - (horizontalOffset * dHeading);
//
//            dX = Math.cos(dHeading) * (number) - arcRad;
//            dY = Math.sin(dHeading) * (number);




        }

        //rotating to absolute coordinates vs robot relative calculated above
        x += Math.cos(h) * dX - Math.sin(h) * dY;
        y += Math.sin(h) * dX + Math.cos(h) * dY;
        h += dHeading;

        if (h>2*Math.PI) {
            h-=2*Math.PI;
        } else if (h<0) {
            h+=2*Math.PI;
        }

        //update reference values to current position
        vlPrevDist = newVerticalLeft;
        vrPrevDist = newVerticalRight;
        hPrevDist = newHorizontal;


    }

    public double[] getLocation(){
//        localize();
        return new double[]{x,y,h};
    }

    public Pose2d getPose(){
        return new Pose2d(x, y, new Rotation2d(h));
    }

    public double[] getRawValues(){
        return new double[]{newVerticalLeft, newVerticalRight, newHorizontal};
    }

    public void setPosition(double x, double y){
        this.x = x;
        this.y = y;
    }

    public void setHeading(double h) {
        this.h = h;
    }
    public double getHeading() {
        return h;
    }

    public void setPose(Pose2d pose){
        this.x = pose.getX();
        this.y = pose.getY();
        this.h = pose.getHeading();
    }

    public void setLateralDistance(double distance) {
        this.LATERAL_DISTANCE = distance;
    }

    public double getLateralDistance() {
        return this.LATERAL_DISTANCE;
    }

}