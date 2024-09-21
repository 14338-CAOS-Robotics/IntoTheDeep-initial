package org.firstinspires.ftc.teamcode.Autonomous;

//import com.arcrobotics.ftclib.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.DrivetrainUtils.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Hardware.CAOSMecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous
public class FarsidePark extends LinearOpMode {
    Robot robot;
    //Movement drive;
    CAOSMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new CAOSMecanumDrive(hardwareMap);
        Pose2d startPos = new Pose2d(-36,55.5, Math.toRadians(-90));
        TrajectorySequence farsideTraj;
        farsideTraj = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -64.50, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-36.00, -12.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(48.00, -12.00, Math.toRadians(90.00)))
                .build();
        drive.setPoseEstimate(farsideTraj.start());
        waitForStart();
        while (opModeIsActive()) {
            drive.followTrajectorySequence(farsideTraj);


            /*
            drive.followPath(drive.getLine(new double[][] {{-36,55.5},{-36,12}}),
                    0.2,0,1,0.05,4,0,false,false );
            drive.followPath(drive.getLine(new double[][] {{-36,12},{48,12}}),
                    0.2,0,1,0.05,4,0,false,false );

             */

        }
    }
}
