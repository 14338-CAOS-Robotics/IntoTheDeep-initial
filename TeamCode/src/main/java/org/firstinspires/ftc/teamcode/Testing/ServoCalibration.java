package org.firstinspires.ftc.teamcode.Testing;


import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.IntakePositions.*;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls;
import org.firstinspires.ftc.teamcode.GamepadControls.Controller;


@Config
public class ServoCalibration extends LinearOpMode {


    public static int armMotorDown = 0-120;
    public static int armMotorUp = 115-120;


    public static boolean wristZero = true;


    @Override
    public void runOpMode() throws InterruptedException {
        setOpMode(this);
        Servo intakeWrist = hardwareMap.servo.get("intakeWrist");
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Controller gp1 = new Controller(gamepad1);
        intakeWrist.setDirection(Servo.Direction.FORWARD);
        intakeWrist.setPosition(intakeWristOpen);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            Controller.update();
            telemetry.addData("arm ticks",armMotor.getCurrentPosition());
            if(gp1.get(ButtonControls.Input.CROSS, ButtonControls.ButtonState.TAP)) {
                if(wristZero) {
                    intakeWrist.setPosition(intakeWristTransfer);
                } else {
                    intakeWrist.setPosition(intakeWristOpen);
                }
                wristZero = !wristZero;
            }
            if(gp1.get(ButtonControls.Input.CIRCLE, ButtonControls.ButtonState.TAP)) {
                if(armMotor.getCurrentPosition() == armMotorDown) {
                    armMotor.setTargetPosition(armMotorUp);
                    armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(.3);
                } else {
                    armMotor.setTargetPosition(armMotorDown);
                    armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(.1);
                }
            }
            telemetry.update();
        }
//        Servo lClaw = hardwareMap.servo.get("leftClaw");
//        Servo rClaw = hardwareMap.servo.get("rightClaw");
//        Distance_Sensor leftIntakeDist = new Distance_Sensor("leftDist");
//        Distance_Sensor rightIntakeDist = new Distance_Sensor("rightDist");
//        Controller gp1 = new Controller(gamepad1);
//        lClaw.setDirection(Servo.Direction.REVERSE);
//        rClaw.setDirection(Servo.Direction.FORWARD);
//        boolean rClawisOpen = false;
//        boolean lClawisOpen = false;
//        //
//
//        int count = 0;
//        waitForStart();
//        while (opModeIsActive() && !isStopRequested()) {
//            leftIntakeDist.distanceUpdate();
//            rightIntakeDist.distanceUpdate();
//            Controller.update();
//            if (gp1.get(ButtonControls.Input.TRIANGLE, ButtonControls.ButtonState.TAP)) {
//                if(!rClawisOpen ^ !lClawisOpen) {
//                    rClaw.setPosition(rClawClosed);
//                    lClaw.setPosition(lClawClosed);
//                    rClawisOpen = false;
//                    lClawisOpen = false;
//
//                } else if(lClawisOpen && rClawisOpen){
//                    rClaw.setPosition(rClawClosed);
//                    lClaw.setPosition(lClawClosed);
//                    rClawisOpen = false;
//                    lClawisOpen = false;
//                } else if(!lClawisOpen && !rClawisOpen) {
//                    rClaw.setPosition(rClawOpen);
//                    lClaw.setPosition(lClawOpen);
//                    rClawisOpen = true;
//                    lClawisOpen = true;
//                }
//            }
//            if(leftIntakeDist.getMM() < leftDistMin && lClawisOpen && leftIntakeDist.isChanging) {
//                lClaw.setPosition(lClawClosed);
//                lClawisOpen = false;
//            }
//            if(rightIntakeDist.getMM() < rightDistMin && rClawisOpen && rightIntakeDist.isChanging) {
//                rClaw.setPosition(rClawClosed);
//                rClawisOpen = false;
//            }
//                telemetry.addData("left distance", leftIntakeDist.getMM());
//                telemetry.addData("right Position", rightIntakeDist.getMM());
//            telemetry.update();
//
//        }
    }
}
