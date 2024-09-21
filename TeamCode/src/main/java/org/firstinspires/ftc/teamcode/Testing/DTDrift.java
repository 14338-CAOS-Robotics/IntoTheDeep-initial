package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Hardware.CAOSMecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

public class DTDrift extends LinearOpMode {
    public static double driveP = 0.2;
    @Override
    public void runOpMode() throws InterruptedException {
        setOpMode(this);
        CAOSMecanumDrive drive = new CAOSMecanumDrive(hardwareMap);
        Robot robot = new Robot();
//        robot.armToGrab();
        waitForStart();
        while(opModeIsActive()){
            drive.update();
            drive.setDrivePower(driveP,0,0,0.5);
            telemetry.addData("current pos", drive.getPoseEstimate());
            telemetry.update();
        }
    }
}
