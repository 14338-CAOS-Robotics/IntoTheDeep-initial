package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls.ButtonState.TAP;
import static org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls.Input.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls;
import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

@TeleOp
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.setOpMode(this);
        Robot robot = new Robot();
        Controller gp1 = new Controller(gamepad1);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            if(gp1.get(TRIANGLE, TAP)) {
                robot.intake.openClaw();
            } else if(gp1.get(SQUARE, TAP)) {
                robot.intake.closeClaw();
            }
        }
    }
}
