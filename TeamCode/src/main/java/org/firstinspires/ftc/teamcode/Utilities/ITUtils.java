package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.roadrunner.geometry.Pose2d;

//@Config
public class ITUtils {


    public static double distancePower = 2;

    public static double ITP = 0;
    public static double ITI = 0;
    public static double ITD = 0;

    public static double kStrafe = 0.001;
    public static double kDrive = 0.001;

    public static double kHDist = 1;

    public static Pose2d junctions[] = new Pose2d[4];

    public static Pose2d cachedPose = new Pose2d(0,0,0);



//    public static void initializeIT() {
//        junctions[0] = new Pose2d(-48, 48, 0);
//        junctions[1] = new Pose2d(-24, 48, 0);
//        junctions[2] = new Pose2d(0, 48, 0);
//        junctions[3] = new Pose2d(24, 48, 0);
//        junctions[4] = new Pose2d(48, 48, 0);
//
//        junctions[5] = new Pose2d(-48, 24, 0);
//        junctions[6] = new Pose2d(-24, 24, 0);
//        junctions[7] = new Pose2d(0, 24, 0);
//        junctions[8] = new Pose2d(24, 24, 0);
//        junctions[9] = new Pose2d(48, 24, 0);
//
//        junctions[10] = new Pose2d(-48, 0, 0);
//        junctions[11] = new Pose2d(-24, 0, 0);
//        junctions[12] = new Pose2d(0, 0, 0);
//        junctions[13] = new Pose2d(24, 0, 0);
//        junctions[14] = new Pose2d(48, 0, 0);
//
//        junctions[15] = new Pose2d(-48, -24, 0);
//        junctions[16] = new Pose2d(-24, -24, 0);
//        junctions[17] = new Pose2d(0, -24, 0);
//        junctions[18] = new Pose2d(24, -24, 0);
//        junctions[19] = new Pose2d(48, -24, 0);
//
//        junctions[20] = new Pose2d(-48, -48, 0);
//        junctions[21] = new Pose2d(-24, -48, 0);
//        junctions[22] = new Pose2d(0, -48, 0);
//        junctions[23] = new Pose2d(24, -48, 0);
//        junctions[24] = new Pose2d(48, -48, 0);
//    }

    public static void initializeIT() {
        junctions[0] = new Pose2d(0, 24, 0);
        junctions[1] = new Pose2d(24, 24, 0);
        junctions[2] = new Pose2d(0, 0, 0);
        junctions[3] = new Pose2d(24, 0, 0);

    }

}


