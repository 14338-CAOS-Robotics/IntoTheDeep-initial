package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

public class AprilTagHeadingFixer {
    public AprilTagReviewer aprilTag;
    Telemetry telemetry;
    List<AprilTagDetection> detections;
    double heading;
    double basisHeading;

    public AprilTagHeadingFixer(HardwareMap hardwareMap, Telemetry telemetry) {
        aprilTag = new AprilTagReviewer(hardwareMap);
        this.telemetry = telemetry;
        detections = aprilTag.getCameraDetections();
        heading = 0;
        basisHeading = 0;
    }
    public void setBasisHeading(double basisHeading) {
        this.basisHeading = basisHeading;
    }
    public double getBasisHeading() {
        return basisHeading;
    }

    public AprilTagHeadingFixer(HardwareMap hardwareMap) {
        aprilTag = new AprilTagReviewer(hardwareMap);
        detections = aprilTag.getCameraDetections();
        heading = 0;
    }
    public void update() {
        detections = aprilTag.getCameraDetections();
    }
    public double getNearestTagHeading(){
        update();
        double leastRange = 9999;
        double newHeading = heading;
        for (AprilTagDetection det : detections) {
            try {
                if (det.ftcPose.range < leastRange) {
                    newHeading = det.ftcPose.yaw;
                    newHeading -= basisHeading;
                    if (det.id > 6) {
                        //are we in degrees at this point?
                        newHeading += 180;
                    }
                    if (newHeading > 360) {
                        newHeading -= 360;
                    } else if (newHeading < 0) {
                        newHeading += 360;
                    }
                    newHeading = Math.toRadians(newHeading);
                }
            } catch(RuntimeException e) {
                return -1;

            }
        }
        if (this.heading != newHeading) {
            this.heading = newHeading;
            return newHeading;
        }
        else {
            return -1;
        }
    }

}
