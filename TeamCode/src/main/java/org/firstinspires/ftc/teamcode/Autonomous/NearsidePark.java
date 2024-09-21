package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DrivetrainUtils.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Hardware.CAOSMecanumDrive;
@Autonomous
public class NearsidePark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CAOSMecanumDrive drive = new CAOSMecanumDrive(hardwareMap);
        TrajectorySequence nearsideTraj = drive.trajectorySequenceBuilder(new Pose2d(12.00, -64.50, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(12.00, -60.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(48.00, -60.00, Math.toRadians(90.00)))
                .build();
        drive.setPoseEstimate(nearsideTraj.start());
        waitForStart();
        while (opModeIsActive()) {
            drive.followTrajectorySequence(nearsideTraj);
        }
    }
}
