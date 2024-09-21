package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.armJointPositionPixel;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.armJointPositionScore;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.armJointPositionTransfer;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.armJointWaitTime;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.armJointWaitTime2;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.clawPositionClosed;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.clawPositionOpen;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.pixelDropperReal;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.pixelDropperRelease;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.slideHangClaw;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.slideHangV4b;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.v4bPositionDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.RobotConstants.v4bPositionUp;

import static org.firstinspires.ftc.teamcode.Hardware.Robot.DriveMode.*;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.OuttakeConstants.outtakeWaitTime;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.OuttakeConstants.wristConstantTime;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.linearOpMode;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Localization.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;
import org.firstinspires.ftc.teamcode.Utilities.Side;
import org.firstinspires.ftc.teamcode.Vision.AprilTagHeadingFixer;

/**
 * A class for containing an FTC Mecanum robot
 */
public class Robot {
    public CAOSMecanumDrive drivetrain;
    public CAOSHardware caosHardware;

    public DriveMode currentMode = FIELD_CENTRIC_DRIVE;
    public Movement movement;
    Controller gp1;
    Controller gp2;
    public Intake intake;
    public Outtake outtake;

    public enum DriveMode {
        FIELD_CENTRIC_DRIVE,
        LOCK_HEADING_DRIVE,
        ROBOT_CENTRIC_DRIVE
    }

    public Robot() {
        initRobot();
    }


    /**
     * Set everything up in all the subsystems
     */
    public void initRobot() {
        drivetrain = new CAOSMecanumDrive(hardwareMap);
        caosHardware = new CAOSHardware(hardwareMap);
        movement = new Movement(0, 0, Math.PI / 2, caosHardware);
        intake = new Intake();
        outtake = new Outtake();
    }

    /**
     * Update PIDs and sensors in all the subsystems
     * RUN ONCE PER LOOP
     */
    public void update() {
    }

    public void setController1(Controller controller) {
        gp1 = controller;
    }

    public void setController2(Controller controller) {
        gp2 = controller;
    }

    public void setDriveMode(DriveMode mode) {
        currentMode = mode;
    }
}


