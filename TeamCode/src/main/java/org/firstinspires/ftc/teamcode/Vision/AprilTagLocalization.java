package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

//TODO: write offset methods for the different tags.
//TODO: Write a class/subclass/method that searches for the tag we want to correlate spikes to the pixel on the board.
public class AprilTagLocalization {
    //current robot position.
    double tagX = 0;
    double tagY = 0;
    double tagH = 0;
    double x = 0;
    double y = 0;
    double h = 0;
    double colesAngle;

    double range = 0;
    double bearing = 0;

    double yaw = 0;

    ArrayList<Double> allX = new ArrayList<>();
    ArrayList<Double> allY = new ArrayList<>();
    ArrayList<Double> allH = new ArrayList<>();
    //    Pose2d currentPose;
    private List<AprilTagDetection> locFrom;
    static AprilTagReviewer aprilTag;
    Telemetry telemetry;

    int id = 0;
    public AprilTagLocalization(HardwareMap hwMap, int[] tags, Telemetry telemetry) {
        aprilTag = new AprilTagReviewer(hwMap);
        locFrom = aprilTag.getCameraDetections();
        this.telemetry = telemetry;
    }

    public static Pose2d getTagPos(int id) {
        List<AprilTagDetection> locFrom = aprilTag.getCameraDetections();
        double[] transformedDetections;
        for (AprilTagDetection detection : locFrom) {
            if (detection.id == id) {
                Pose2d roboPose = new Pose2d(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw);
                return offsetToCenter(roboPose);
            }
        }
        throw new ArithmeticException("Oh crap!");
    }

    public void localize() {
        allX.clear();
        allY.clear();
        allH.clear();
        locFrom = aprilTag.getCameraDetections();
        double[] transformedDetection;
        for (AprilTagDetection detection : locFrom) {
            if ((AprilTagReviewer.TagFamily.contains(detection.id)||true) && detection.metadata != null) {
                //NOTE: ftcPose = camera centric.
                //position like odometry
                //removed the z axis to streamline performance. Also with the assumption its unnecessary.
                    //yaw = detection.ftcPose.yaw;

                //accountOffsets(detection.id);

                telemetry.addLine("Jolly! I've found an AprilTag!");
                tagY=detection.ftcPose.y;
                yaw=detection.ftcPose.yaw;
                tagX=detection.ftcPose.x;

                bearing = detection.ftcPose.bearing;
                range = detection.ftcPose.range;

               /* transformedDetection = tagToField(bearing, yaw, range);

                x = transformedDetection[0];
                y = transformedDetection[1];
                h = transformedDetection[2];*/

                //offsetToCenter();

                allX.add(x);
                allY.add(y);
                allH.add(h);
                //rotation? pitch, roll, yaw
                //y is assumed as the yaw(heading)
                //(detection.ftcPose.yaw);

                this.id = detection.id;
            }
//            accountOffsets(this.id);

        }
        //calculateAverage();



    }
    private static Pose2d offsetToCenter(Pose2d roboCentric) {
        double x_off = 7.41;
        double y_off = 10.505;

        /*
        double mx = a*Math.cos(Math.toRadians(h)) - b*Math.sin(Math.toRadians(h));
        //this.x -= 6.1*Math.cos(Math.toRadians(h));
        double my = a*Math.sin(Math.toRadians(h)) + b*Math.cos(Math.toRadians(h));

        telemetry.addData("Matrix x", mx);
        telemetry.addData("Matrix y", my);

        x+= mx;
        y+= my;
         */
        double tx = roboCentric.getX()+x_off;
        double ty = roboCentric.getY()+y_off;

        double new_bearing = Math.atan(tx/ty);
        double new_range = Math.sqrt(tx*tx+ty*ty);

        //If this breaks, put it in radians
        double colesSecondAngle = roboCentric.getHeading() + new_bearing;

        double x = new_range * Math.sin(colesSecondAngle);
        double y = new_range * Math.cos(colesSecondAngle);
        double h = roboCentric.getHeading();
        return new Pose2d(x,y,h);
    }


    private void calculateAverage() {
            x = allX.stream()
                    .mapToDouble(d -> d)
                    .average()
                    .orElse(0.0);
            y = allY.stream()
                    .mapToDouble(d -> d)
                    .average()
                    .orElse(0.0);
            h = allH.stream()
                    .mapToDouble(d -> d)
                    .average()
                    .orElse(0.0);
    }




    private void accountOffsets(int tagNum) {
        //accountCamOffsets();
        //checks if side is red. If not, it is blue.
        /*
        if (Side.getSide().equals("red")) {
            switch (tagNum) {
                //blue backdrop
                case 1:
                    allX.set(allX.size()-1,RED_BLUE_BACKDROP_LEFT[0]+allX.get(allX.size()-1));
                    allY.set(allY.size()-1,RED_BLUE_BACKDROP_LEFT[1]-allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+RED_BLUE_BACKDROP_LEFT[2]);
                    break;
                case 2:
                    allX.set(allX.size()-1,RED_BLUE_BACKDROP_MIDDLE[0]+allX.get(allX.size()-1));
                    allY.set(allY.size()-1,RED_BLUE_BACKDROP_MIDDLE[1]-allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+RED_BLUE_BACKDROP_MIDDLE[2]);
                    break;
                case 3:
                    allX.set(allX.size()-1,RED_BLUE_BACKDROP_RIGHT[0]+allX.get(allX.size()-1));
                    allY.set(allY.size()-1,RED_BLUE_BACKDROP_RIGHT[1]-allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+RED_BLUE_BACKDROP_RIGHT[2]);
                    break;

                //red backdrop
                case 4:
                    allX.set(allX.size()-1,RED_RED_BACKDROP_LEFT[0]+allX.get(allX.size()-1));
                    allY.set(allY.size()-1,RED_RED_BACKDROP_LEFT[1]-allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+RED_RED_BACKDROP_LEFT[2]);
                    break;
                case 5:
                    allX.set(allX.size()-1,RED_RED_BACKDROP_MIDDLE[0]+allX.get(allX.size()-1));
                    allY.set(allY.size()-1,RED_RED_BACKDROP_MIDDLE[1]-allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+RED_RED_BACKDROP_MIDDLE[2]);
                    break;
                case 6:
                    allX.set(allX.size()-1,RED_RED_BACKDROP_RIGHT[0]+allX.get(allX.size()-1));
                    allY.set(allY.size()-1,RED_RED_BACKDROP_RIGHT[1]-allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+RED_RED_BACKDROP_RIGHT[2]);
                    break;

                //blue Atag
                case 9:
                    allX.set(allX.size()-1,RED_BLUE_APRILTAG_SMALL[0]-allX.get(allX.size()-1));
                    allY.set(allY.size()-1,RED_BLUE_APRILTAG_SMALL[1]+allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+RED_BLUE_APRILTAG_SMALL[2]);
                    break;
                case 10:
                    allX.set(allX.size()-1,RED_BLUE_APRILTAG_LARGE[0]-allX.get(allX.size()-1));
                    allY.set(allY.size()-1,RED_BLUE_APRILTAG_LARGE[1]+allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+RED_BLUE_APRILTAG_LARGE[2]);
                    break;

                //red Atag
                case 7:
                    allX.set(allX.size()-1,RED_RED_APRILTAG_SMALL[0]-allX.get(allX.size()-1));
                    allY.set(allY.size()-1,RED_RED_APRILTAG_SMALL[1]+allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+RED_RED_APRILTAG_SMALL[2]);
                    break;
                case 8:
                    allX.set(allX.size()-1,RED_RED_APRILTAG_LARGE[0]-allX.get(allX.size()-1));
                    allY.set(allY.size()-1,RED_RED_APRILTAG_LARGE[1]+allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+RED_RED_APRILTAG_LARGE[2]);
                    break;
            }
        } else {
            switch (tagNum) {
                //blue backdrop
                case 1:
                    allX.set(allX.size()-1,allX.get(allX.size()-1)-BLUE_BLUE_BACKDROP_LEFT[0]);
                    allY.set(allY.size()-1,BLUE_BLUE_BACKDROP_LEFT[1]+allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+BLUE_BLUE_BACKDROP_LEFT[2]);
                    break;
                case 2:
                    allX.set(allX.size()-1,allX.get(allX.size()-1)-BLUE_BLUE_BACKDROP_MIDDLE[0]);
                    allY.set(allY.size()-1,BLUE_BLUE_BACKDROP_MIDDLE[1]+allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+BLUE_BLUE_BACKDROP_MIDDLE[2]);
                    break;
                case 3:
                    allX.set(allX.size()-1,allX.get(allX.size()-1)-BLUE_BLUE_BACKDROP_RIGHT[0]);
                    allY.set(allY.size()-1,BLUE_BLUE_BACKDROP_RIGHT[1]+allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+BLUE_BLUE_BACKDROP_RIGHT[2]);
                    break;

                //red backdrop
                case 4:
                    allX.set(allX.size()-1,allX.get(allX.size()-1)-BLUE_RED_BACKDROP_LEFT[0]);
                    allY.set(allY.size()-1,BLUE_RED_BACKDROP_LEFT[1]+allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+BLUE_RED_BACKDROP_LEFT[2]);
                    break;
                case 5:
                    allX.set(allX.size()-1,allX.get(allX.size()-1)-BLUE_RED_BACKDROP_MIDDLE[0]);
                    allY.set(allY.size()-1,BLUE_RED_BACKDROP_MIDDLE[1]+allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+BLUE_RED_BACKDROP_MIDDLE[2]);
                    break;
                case 6:
                    allX.set(allX.size()-1,allX.get(allX.size()-1)-BLUE_RED_BACKDROP_RIGHT[0]);
                    allY.set(allY.size()-1,BLUE_RED_BACKDROP_RIGHT[1]+allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+BLUE_RED_BACKDROP_RIGHT[2]);
                    break;

                //blue Atag
                case 9:
                    allX.set(allX.size()-1,BLUE_BLUE_APRILTAG_SMALL[0]+allX.get(allX.size()-1));
                    allY.set(allY.size()-1,BLUE_BLUE_APRILTAG_SMALL[1]-allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+BLUE_BLUE_APRILTAG_SMALL[2]);
                    break;
                case 10:
                    allX.set(allX.size()-1,BLUE_BLUE_APRILTAG_LARGE[0]+allX.get(allX.size()-1));
                    allY.set(allY.size()-1,BLUE_BLUE_APRILTAG_LARGE[1]-allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+BLUE_BLUE_APRILTAG_LARGE[2]);
                    break;

                //red Atag
                case 7:
                    allX.set(allX.size()-1,BLUE_RED_APRILTAG_SMALL[0]+allX.get(allX.size()-1));
                    allY.set(allY.size()-1,BLUE_RED_APRILTAG_SMALL[1]-allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+BLUE_RED_APRILTAG_SMALL[2]);
                    break;
                case 8:
                    allX.set(allX.size()-1,BLUE_RED_APRILTAG_LARGE[0]+allX.get(allX.size()-1));
                    allY.set(allY.size()-1,BLUE_RED_APRILTAG_LARGE[1]-allY.get(allY.size()-1));
                    allH.set(allH.size()-1,allH.get(allH.size()-1)+BLUE_RED_APRILTAG_LARGE[2]);
                    break;
            }
        }*/
    }

    private double[] tagToField(double bearing, double yaw, double range) {
        //colesAngle aka "heading"
        colesAngle = Math.toRadians(yaw);

        double fx = range * Math.sin(colesAngle);
        double fy = range * Math.cos(colesAngle);
        telemetry.addData("Target x", fx);
        telemetry.addData("Target y", fy);
        double fh = Math.toDegrees(colesAngle);

        return new double[] {fx, fy, fh};
    }


    public List<AprilTagDetection> getDetections() {
        return locFrom;
    }

    public double[] getPose () {
        return new double[]{x,y,h};
    }
    public double getX () {
        return x;
    }

    public double getY () {
        return y;
    }

    public double getH () {
        return h;
    }

    //WHEN USING: MAKE SURE TO CHECK IF COORDS ARE 0,0,0 THEN CONTINUE ON THE CURRENT PATH!
    public double[] getHomingCoords(int tagNum, int offset) {
        for (AprilTagDetection detection : locFrom) {
            if (detection.id == tagNum) {
                return new double[]{detection.ftcPose.x, detection.ftcPose.y-offset, detection.ftcPose.yaw};
            }
        }
        return new double[]{0,0,0};
    }

    }