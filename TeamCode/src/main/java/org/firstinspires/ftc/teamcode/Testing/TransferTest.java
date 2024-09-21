package org.firstinspires.ftc.teamcode.Testing;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls;
import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

import static org.firstinspires.ftc.teamcode.Hardware.Robot.OuttakeConstants.*;

//@Config
//@TeleOp
public class TransferTest extends LinearOpMode {


    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.setOpMode(this);
        robot = new Robot();
        Controller gp1 = new Controller(gamepad1);
        robot.intake.openClaw();
        boolean transfering = false;
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            Controller.update();
            robot.intake.update();
            if(gp1.get(ButtonControls.Input.CROSS, ButtonControls.ButtonState.TAP)) {
                robot.outtake.cycleV4b();
            } else if(gp1.get(ButtonControls.Input.RB1, ButtonControls.ButtonState.TAP)) {
                robot.intake.cycleClaw();
            }else if(gp1.get(ButtonControls.Input.LB1, ButtonControls.ButtonState.TAP)) {
                robot.outtake.cycleOuttakeClaw();
            } else if(gp1.get(ButtonControls.Input.CIRCLE, ButtonControls.ButtonState.TAP)) {
                robot.outtake.cycleWrist();
            } else if(gp1.get(ButtonControls.Input.SQUARE, ButtonControls.ButtonState.TAP)) {
                if (!transfering) {
                    robot.intake.setArmState(Intake.ArmStates.TRANSFERING);
                    transfering = true;
                } else {
                    robot.intake.setArmState(Intake.ArmStates.ZERO);
                    transfering = false;
                }
            }
            telemetry.update();
        }
    }
}
