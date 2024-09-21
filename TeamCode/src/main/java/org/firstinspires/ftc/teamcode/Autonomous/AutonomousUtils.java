package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Vision.AprilTagReviewer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class AutonomousUtils {
    Movement movement;
    Robot robot;
    Telemetry telemetry;
    AprilTagReviewer aprilTag;
    List<AprilTagDetection> detections;


    public AutonomousUtils(Movement movement, Robot robot, HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.robot = robot;
        this.movement = movement;
        aprilTag = robot.colesHeadingFixer.aprilTag;
        update();

    }
    public boolean goRelativeToAprilTag(int id, double x, double y, double heading, double posThreshold, double headingThreshold) {
        AprilTagDetection currentDetection = aprilTag.getDetection(id);
        if (currentDetection == null) {
            return false;
        }
        telemetry.addData("tag found", "yep");
        telemetry.update();
        try {
            telemetry.addData("Heading lock:", "in progress");
            telemetry.update();
            while ((Math.toRadians(currentDetection.ftcPose.yaw) - heading) % (2*Math.PI) > headingThreshold) {
                movement.odo.setHeading(-currentDetection.ftcPose.yaw);
                movement.snapHeading(movement.odo.getHeading() - Math.toRadians(currentDetection.ftcPose.yaw), headingThreshold);
                currentDetection = aprilTag.getDetection(id);
                telemetry.addData("Heading:", movement.odo.getHeading());
                telemetry.addData("target:", heading);
                telemetry.update();
            }
            telemetry.addData("Heading lock:", "Done");
            telemetry.update();
            double dx = posThreshold + 1, dy = posThreshold + 1;
            while (Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2)) > posThreshold) {
                dx = (x-currentDetection.ftcPose.x);
                dy = (y-currentDetection.ftcPose.y);
                Movement.Function[] path = movement.getLine(new double[][] {{0,0},{dx,dy}});

                while(movement.followPath(path, 0.5, movement.odo.getHeading(),1,1,posThreshold,0,false,false)){}
            }
        } catch (NullPointerException e) {
            return false;
        }

        return true;
        //return false;
    }

    public void update() {
        detections = aprilTag.getCameraDetections();
    }
}
