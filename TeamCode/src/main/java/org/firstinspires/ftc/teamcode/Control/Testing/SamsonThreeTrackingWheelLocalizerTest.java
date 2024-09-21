package org.firstinspires.ftc.teamcode.Control.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.Localization.ThreeTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Control.Localization.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls;
import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.GamepadControls.JoystickControls;
import org.firstinspires.ftc.teamcode.Hardware.CAOSMecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;
import org.firstinspires.ftc.teamcode.Hardware.Physical.Hardware;
import org.firstinspires.ftc.teamcode.Vision.AprilTagHeadingFixer;

import java.util.ArrayList;
import java.util.Arrays;
@TeleOp
public class SamsonThreeTrackingWheelLocalizerTest extends LinearOpMode {
    public long lastLoop;
//        Arrays.asList(
//            new Pose2d(0,0,Math.toRadians(90)),
//            new Pose2d(0,0,Math.toRadians(90)),
//            new Pose2d(0,0,Math.toRadians(90))
//    );
    public double joystickScaler(double inp){
        if (inp <= 0) return 0;
        else if (inp <= 0.5) return inp * inp;
        else if (inp <= 1) return 1.5*inp - 0.5;
        else return 1;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagHeadingFixer colesHeadingFixer = new AprilTagHeadingFixer(hardwareMap, telemetry);
        Hardware hardware = new CAOSHardware(hardwareMap);
        CAOSMecanumDrive caosMecanumDrive = new CAOSMecanumDrive(hardwareMap);
        ThreeWheelOdometry odo = new ThreeWheelOdometry(0.0,0,0, hardware);
        hardware.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Controller gp1 = new Controller(gamepad1);
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();
        lastLoop = System.nanoTime();
        while(opModeIsActive()) {
            colesHeadingFixer.update();
            Controller.update();
            odo.localize();
            double newHeading = colesHeadingFixer.getNearestTagHeading();
            if (newHeading > 0) odo.setHeading(newHeading);
            double[] Pos = odo.getLocation();
            double[] Raw = odo.getRawValues();
            caosMecanumDrive.setFieldCentricDrivePower(
                    gp1.get(JoystickControls.Input.LEFT, JoystickControls.Value.INVERT_X),
                    gp1.get(JoystickControls.Input.LEFT, JoystickControls.Value.Y),
                    gp1.get(JoystickControls.Input.RIGHT, JoystickControls.Value.X),
                    Pos[2]);
            telemetry.addData("X", Pos[0]);
            telemetry.addData("Y", Pos[1]);
            telemetry.addData("Heading", Pos[2]);
            telemetry.addData("Veritical Left Encoder", Raw[0]);
            telemetry.addData("Vertical Right Encoder", Raw[1]);
            telemetry.addData("Horizontal Encoder", Raw[2]);
            telemetry.update();

        }
    }
}
