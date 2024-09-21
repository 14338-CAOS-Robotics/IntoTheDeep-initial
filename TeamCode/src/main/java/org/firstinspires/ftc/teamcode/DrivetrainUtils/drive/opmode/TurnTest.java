package org.firstinspires.ftc.teamcode.DrivetrainUtils.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.CAOSMecanumDrive;


/*
 * This is a simple routine to test turning capabilities.
 */
@Disabled
@Autonomous(name = "turn test", group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        CAOSMecanumDrive drive = new CAOSMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
