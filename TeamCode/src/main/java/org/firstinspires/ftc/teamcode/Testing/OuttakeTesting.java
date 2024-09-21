package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Hardware.Robot.OuttakeConstants.*;

import android.text.style.TextAppearanceSpan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls;
import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

public class OuttakeTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.setOpMode(this);
        Servo l4b = hardwareMap.servo.get("l4b");
        Servo r4b = hardwareMap.servo.get("r4b");

        Servo armJoint = hardwareMap.servo.get("aj");

        Servo rightOuttakeClaw = hardwareMap.servo.get("roc");
        Servo leftOuttakeClaw = hardwareMap.servo.get("loc");
        r4b.setDirection(Servo.Direction.REVERSE);
        rightOuttakeClaw.setDirection(Servo.Direction.REVERSE);
        Controller gp1 = new Controller(gamepad1);
        boolean v4bZero = false;
        boolean wristZero = true;
        boolean clawZero = true;
        Robot robot = new Robot();
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            Controller.update();
            if(gp1.get(ButtonControls.Input.CROSS, ButtonControls.ButtonState.TAP)) {
                if (!v4bZero) {
                    l4b.setPosition(armZero);
                    r4b.setPosition(armZero);
                    telemetry.addData("4b state", "zero");
                    v4bZero = true;
                } else {
                    l4b.setPosition(armExtend);
                    r4b.setPosition(armExtend);
                    telemetry.addData("4b state", "extend");
                    v4bZero = false;
                }
            } else if(gp1.get(ButtonControls.Input.TRIANGLE, ButtonControls.ButtonState.TAP)) {
                if(wristZero) {
                    armJoint.setPosition(wristExtend);
                    telemetry.addData("wrist state", "extend");
                    wristZero = false;
                } else {
                    armJoint.setPosition(Robot.OuttakeConstants.wristZero);
                    telemetry.addData("wrist state", "zero");
                    wristZero = true;
                }
            } else if(gp1.get(ButtonControls.Input.CIRCLE, ButtonControls.ButtonState.TAP)) {
                if(clawZero) {
                    leftOuttakeClaw.setPosition(lOCOpen);
                    rightOuttakeClaw.setPosition(rOCOpen);
                    telemetry.addData("claw state", "open");
                    clawZero = false;
                } else {
                    leftOuttakeClaw.setPosition(lOCClose);
                    rightOuttakeClaw.setPosition(rOCClose);
                    telemetry.addData("claw state", "close");
                    clawZero = true;
                }
            } else if(gp1.get(ButtonControls.Input.SQUARE, ButtonControls.ButtonState.TAP)) {
                robot.outtake.cycleV4b();
            }
            telemetry.update();
        }
    }
}
