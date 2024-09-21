package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Control.MathUtils;

import java.util.Arrays;

public class PathPoint {
    private double heading;
    private double x;
    private double y;

    public PathPoint() {
        this.setPose(0,0,0);
    }

    public PathPoint(double x, double y, double heading) {
        this.setPose(x,y,heading);
    }

    public PathPoint(double[] pos, double heading) {
        this.setPose(pos, heading);
    }

    public PathPoint(Pose2d pose) {
        this.setPose(pose);
    }

    public void setHeading (double heading) {
        heading = heading % 2*Math.PI;
        this.heading = heading;
    }

    public void setHeading (Pose2d pose) {
        this.heading = pose.getHeading();
    }

    public void setPosition(double[] pos) {
        this.x = pos[0];
        this.y = pos[1];
    }

    public void setPosition(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setPosition(Pose2d pose){
        this.x = pose.getX();
        this.y = pose.getY();
    }

    public void setPose(double[] pos, double heading) {
        this.setPosition(pos);
        this.setHeading(heading);
    }

    public void setPose(double x, double y, double heading) {
        this.setPosition(x, y);
        this.setHeading(heading);
    }

    public void setPose(Pose2d pose) {
        this.setPosition(pose.getX(), pose.getY());
        this.setHeading(pose.getHeading());
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public double getHeading() {
        return this.heading;
    }

    public double[] getPos() {
        return new double[] {x,y};
    }

    public Pose2d getPose() {
        return new Pose2d(x,y,heading);
    }

    public double getSum() {
        return x+y+Math.toDegrees(heading+Math.atan2(x,y));
    }

    @NonNull
    @Override
    public String toString() {
        return Arrays.toString(this.getPos()) + ", Heading: " + getHeading() + "radians";
    }

}
