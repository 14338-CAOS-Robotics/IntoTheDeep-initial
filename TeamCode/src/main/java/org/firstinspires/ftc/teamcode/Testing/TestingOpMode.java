package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls;
import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID;

public class TestingOpMode extends LinearOpMode {


//    CAOSHardware hardware;
//    ArmControl armControl;
//
//    Servo servo;
//    Controller gp1;
//    @Override
//    public void init() {
//        OpModeUtils.setOpMode(this);
//        hardware = new CAOSHardware(hardwareMap);
//         armControl = new ArmControl(hardware);
//         gp1 = new Controller(gamepad1);
//    }
//
//    boolean transfered = false;
//    @Override
//    public void loop() {
//        Controller.update();
//        armControl.update();
//        if(gp1.get(ButtonControls.Input.CROSS, ButtonControls.ButtonState.TAP)) {
//            if (!transfered) {
//                armControl.setposition(transfering);
//                transfered = true;
//            } else {
//                armControl.setposition(zero);
//                transfered = false;
//            }
//        }
//    }


    CAOSHardware caosHardware;
    Robot robot;

    Controller gp1;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.setOpMode(this);
        robot = new Robot();
        gp1 = new Controller(gamepad1);
    waitForStart();
    while (opModeIsActive() && !isStopRequested()) {
        Controller.update();
        robot.intake.update();
        if (gp1.get(ButtonControls.Input.CROSS, ButtonControls.ButtonState.TAP)) {
            robot.intake.goToArmZero();
            telemetry.update();
        }
        if (gp1.get(ButtonControls.Input.TRIANGLE, ButtonControls.ButtonState.TAP)) {
            robot.intake.goToArmTransfer();
        }
        if (gp1.get(ButtonControls.Input.SQUARE, ButtonControls.ButtonState.TAP)) {
            robot.intake.openClaw();
        }
        if (gp1.get(ButtonControls.Input.CIRCLE, ButtonControls.ButtonState.TAP)) {
            robot.intake.closeClaw();
        }
        telemetry.update();
    }
    }
}
