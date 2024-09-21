package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.TeleOp.MovementTuneTeleOp.MoveTune.*;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Control.MovementTune;
import org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls;
import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.PID;

//@TeleOp(name="custom move tuner", group="TeleOp Linear Opmode")
public class MovementTuneTeleOp extends LinearOpMode {

    @Config
    public static class MoveTune{
        public static boolean changeFunction = false;
        public static double x;
        public static double y;
        public static double h;
    }

    public FtcDashboard dashboard = FtcDashboard.getInstance();

    Robot robot;
//    private PID rotationPID;
    private Movement movement;
    private ElapsedTime runtime = new ElapsedTime();
    Controller controller1;
    CAOSHardware caosHardware;

    Movement.Function[] function;
    public void initialize() {
        robot = new Robot();
        controller1 = new Controller(gamepad1);
//        rotationPID = new PID(proportionalWeight, integralWeight, derivativeWeight);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {
        setOpMode(this);
        caosHardware = new CAOSHardware(hardwareMap);
        movement = new Movement(0,0,0,caosHardware);

        initialize();

        double[] currentLocation = movement.odo.getLocation();
        //function = movement.getLine(new double[][]{{currentLocation[0], currentLocation[1]}, {x, y}});
        function = movement.getBezierCurve(new double[][] {{0,0}, {0,10}, {10,10}, {10,0}});
        waitForStart();

        //To change speed, pass regulateSpeed1(*whateverspeedyouwant*) as an argument of Pose2D, followed by regulateSpeed2()


        while (opModeIsActive() && !isStopRequested()) {
            movement.setMovementPID();
            movement.setHeadingPID();


            if(changeFunction) {
                //function = movement.getLine(new double[][]{{currentLocation[0], currentLocation[1]}, {x, y}});
                changeFunction = false;
                while(!isStopRequested() && movement.followPath(function, 0.5, h, 1, 0.005, 2, 0.1, true, true)){
                    movement.setMovementPID();
                    movement.setHeadingPID();
                    double[] curPos = movement.odo.getLocation();
                    robot.drivetrain.setPoseEstimate(new Pose2d(curPos[0], curPos[1], curPos[2]));
                    multTelemetry.addData("x", curPos[0]);
                    multTelemetry.addData("y", curPos[1]);
                    multTelemetry.addData("heading", curPos[2]);
//                    TelemetryPacket packet = new TelemetryPacket();
//                    packet.fieldOverlay()
//                            .setStrokeWidth(1)
//                            .setFill("blue").fillCircle(curPos[0], curPos[1], 1);
//                    dashboard.sendTelemetryPacket(packet);
                    TelemetryPacket packet2 = new TelemetryPacket();
                    double[] propError = getProportionalError(curPos);
                    packet2.put("xError", propError[0]);
                    packet2.put("yError", propError[1]);
                    packet2.put("hError", propError[2]);
                    double[] derError = getDerivativeError(propError);
                    packet2.put("xDerivative", derError[0]);
                    packet2.put("yDerivative", derError[1]);
                    packet2.put("hDerivative", derError[2]);
                    dashboard.sendTelemetryPacket(packet2);
                    multTelemetry.update();
                }
//                while(!isStopRequested()){
//                    movement.holdPosition(x,y,h);
//                    movement.setMovementPID();
//                    movement.setHeadingPID();
//                    double[] curPos = movement.odo.getLocation();
//                    robot.drivetrain.setPoseEstimate(new Pose2d(curPos[0], curPos[1], curPos[2]));
//                    multTelemetry.addData("x", curPos[0]);
//                    multTelemetry.addData("y", curPos[1]);
//                    multTelemetry.addData("heading", curPos[2]);
////                    TelemetryPacket packet = new TelemetryPacket();
////                    packet.fieldOverlay()
////                            .setStrokeWidth(1)
////                            .setFill("blue").fillCircle(curPos[0], curPos[1], 1);
////                    dashboard.sendTelemetryPacket(packet);
//                    TelemetryPacket packet2 = new TelemetryPacket();
//                    double[] propError = getProportionalError(curPos);
//                    packet2.put("xError", propError[0]);
//                    packet2.put("yError", propError[1]);
//                    packet2.put("hError", propError[2]);
//                    double[] derError = getDerivativeError(propError);
//                    packet2.put("xDerivative", derError[0]);
//                    packet2.put("yDerivative", derError[1]);
//                    packet2.put("hDerivative", derError[2]);
//                    dashboard.sendTelemetryPacket(packet2);
//                    multTelemetry.update();
//                }
            }




        }

    }
    public double[] getProportionalError(double[] currentLocation) {
        double targetX = function[0].get(1); // target x
        double targetY = function[1].get(1); // target y
        double targetHeading = h; // target heading

        double currentX = currentLocation[0]; // current x
        double currentY = currentLocation[1]; // current y
        double currentHeading = movement.odo.getLocation()[2]; // current heading

        double errorX = targetX - currentX; // x error
        double errorY = targetY - currentY; // y error
        double errorHeading = targetHeading - currentHeading; // heading error
        return new double[]{errorX, errorY, errorHeading};
    }


    double previousErrorX = 0; // initialize outside the loop
    double previousErrorY = 0; // initialize outside the loop
    double previousErrorHeading = 0; // initialize outside the loop
    public double[] getDerivativeError(double[] error) {
        double errorDerivativeX = error[0] - previousErrorX; // derivative of x error
        double errorDerivativeY = error[1] - previousErrorY; // derivative of y error
        double errorDerivativeHeading = error[2] - previousErrorHeading; // derivative of heading error

        previousErrorX = error[0]; // update for next iteration
        previousErrorY = error[1]; // update for next iteration
        previousErrorHeading = error[2]; // update for next iteration
        return new double[]{errorDerivativeX, errorDerivativeY, errorDerivativeHeading};
    }

}