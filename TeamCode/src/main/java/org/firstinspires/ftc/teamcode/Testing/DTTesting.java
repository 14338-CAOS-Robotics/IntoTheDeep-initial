package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls;
import org.firstinspires.ftc.teamcode.GamepadControls.JoystickControls;
import org.firstinspires.ftc.teamcode.Hardware.CAOSMecanumDrive;
import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Side;
import org.opencv.core.Range;

public class DTTesting extends LinearOpMode {
    @Config
    public static class DTDeadZone {
        public static double lowerDZ = .01;
    }
    CAOSMecanumDrive caosMecanumDrive;
    Controller gp1;
    boolean needShake;

    @Override
    public void runOpMode() throws InterruptedException {

        setOpMode(this);
        caosMecanumDrive = new CAOSMecanumDrive(hardwareMap);
        CAOSHardware caosHardware = new CAOSHardware(hardwareMap);
        gp1 = new Controller(gamepad1);
        Movement movement = new Movement(0,0,0, caosHardware);
        Robot robot = new Robot();
        robot.intake.goToArmTransfer();
        Side.setRed();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            Controller.update();
            robot.update();
            if(gp1.get(ButtonControls.Input.DPAD_UP, ButtonControls.ButtonState.TAP)) {
                robot.setDriveMode(Robot.DriveMode.FIELD_CENTRIC_DRIVE);
            } else if(gp1.get(ButtonControls.Input.DPAD_DN, ButtonControls.ButtonState.TAP)) {
                robot.setDriveMode(Robot.DriveMode.LOCK_HEADING_DRIVE);
            } else if(gp1.get(ButtonControls.Input.DPAD_L, ButtonControls.ButtonState.TAP)) {
                robot.setDriveMode(Robot.DriveMode.ROBOT_CENTRIC_DRIVE);
            }
            double x = gp1.get(JoystickControls.Input.LEFT, JoystickControls.Value.INVERT_X);
            double y = gp1.get(JoystickControls.Input.LEFT, JoystickControls.Value.Y);
            double h = gp1.get(JoystickControls.Input.RIGHT, JoystickControls.Value.X);
            if(((x <= DTDeadZone.lowerDZ && x >= -DTDeadZone.lowerDZ) && (y <= DTDeadZone.lowerDZ && y >= -DTDeadZone.lowerDZ) && (h <= DTDeadZone.lowerDZ && h >= -DTDeadZone.lowerDZ)) || robot.currentMode == Robot.DriveMode.LOCK_HEADING_DRIVE) {
                double realH = robot.colesHeadingFixer.getNearestTagHeading();
                if(realH > 0) {
                    robot.setHeading(realH);
                    needShake = true;
                }
            } else {
                needShake = false;
            }
            if(needShake) {
                gp1.rumble(.65, 150);
            }
            robot.setDrivePower(x, y, h);
            telemetry.addData("x", robot.Pos[0]);
            telemetry.addData("y", robot.Pos[1]);
            telemetry.addData("h", robot.Pos[2]);
            telemetry.update();

        }
    }
}
