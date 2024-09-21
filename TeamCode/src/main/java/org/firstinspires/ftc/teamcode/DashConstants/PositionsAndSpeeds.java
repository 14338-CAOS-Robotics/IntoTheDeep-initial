package org.firstinspires.ftc.teamcode.DashConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PositionsAndSpeeds {

    @Config
    public static class outtakePositions {
        public static double leftOuttake4bDeployed = 0.0,
                            leftOuttake4bRetracted = 0.0,
                            rightOuttake4bDeployed = 0.0,
                            rightOuttake4bRetracted = 0.0,
                            tiltOuttakeUp = 0.0,
                            tiltOuttakeDown = 0.0,
                            leftOuttakeUp = 0.0,
                            leftOuttakeDown = 0.0,
                            rightOuttakeUp = 0.0,
                            rightOuttakeDown = 0.0;
    }

    @Config
    public static class SlidePositions {


        public static double SlidesP = .003, SlidesI = 0, SlidesD = .0002, F = .3;
    }
    @Config
    public static class IntakePositions {
        public static double intakeWristOpen = .62;
        public static double intakeWristTransfer = 1;
        public static double constantIg = .1;
        public static int transfering = 122-120;
        public static int zero = 5-120;
        public static int zeroOffset = 10;
//        public static int armWaitMillis = 200;
        public static double armZeroAngle = .1;
        public static double kS = 0;
        public static double kV = 0;
        //        public static double kA = 0;
        public static double kCos = 0.3;


        //another hacky fix
        public static double angleMod = 1.1;
    }

    @Config
    public static class RobotConstants {
        public static double v4bPositionUp = 0.2;
        public static double v4bPositionDown = 0.042;
        public static double armJointPositionTransfer = 0;
        public static double armJointPositionScore = 0.47;
        public static double armJointPositionPixel = 0.57;
        public static double clawPositionOpen = 0.22;
        public static double clawPositionClosed = 0.55;
        public static double pixelDropperRelease = 0.69;
        public static double pixelDropperTele = 0;
        public static double pixelDropperReal = 0.2;
        public static double slideHangV4b = 0.7;
        public static double slideHangClaw = 0.4;


        public static double armJointWaitTime = 0.2;
        public static double armJointWaitTime2 = 0.1;
        public static int maxSlideTicks = 2500;
    }



}

