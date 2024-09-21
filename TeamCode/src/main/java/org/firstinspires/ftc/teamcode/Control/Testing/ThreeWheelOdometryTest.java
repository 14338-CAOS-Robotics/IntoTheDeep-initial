package org.firstinspires.ftc.teamcode.Control.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.Localization.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;
import org.firstinspires.ftc.teamcode.Hardware.Physical.Hardware;
@TeleOp
public class ThreeWheelOdometryTest extends LinearOpMode {

    public long lastLoop;

    public void runOpMode() {
        Hardware hardware = new CAOSHardware(hardwareMap);
        ThreeWheelOdometry odo = new ThreeWheelOdometry(0,0,0, hardware);
        ThreeWheelOdometry odo2 = new ThreeWheelOdometry(0,0,0, hardware);
        hardware.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();
        lastLoop = System.nanoTime();

        while(opModeIsActive()) {

            double[] Pos = odo.getLocation();
            double[] Pos2 = odo2.getLocation();
            double[] Raw = odo.getRawValues();
//            double dTime = System.nanoTime() - lastLoop;
//            lastLoop = System.nanoTime();
//            double hz = 1000000000.0 / (dTime);
//            telemetry.addData("Heartz <3", hz);
            telemetry.addData("X", Pos[0]);
            telemetry.addData("Y", Pos[1]);
            telemetry.addData("Heading", Pos[2]);
            telemetry.addData("\t", " ");
            telemetry.addData("X2", Pos2[0]);
            telemetry.addData("Y2", Pos2[1]);
            telemetry.addData("Heading2", Pos2[2]);
            telemetry.addData("Veritical Left Encoder", Raw[0]);
            telemetry.addData("Vertical Right Encoder", Raw[1]);
            telemetry.addData("Horizontal Encoder", Raw[2]);
            telemetry.update();
            odo.localize();


        }

    }



    private void initialize() {

        telemetry.addData("status","initialized");
        telemetry.update();
    }
}
