package org.firstinspires.ftc.teamcode.Hardware;


import static org.firstinspires.ftc.teamcode.Hardware.Robot.DriveMode.*;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;


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
        intake = new Intake(caosHardware);
        outtake = new Outtake(caosHardware);
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


