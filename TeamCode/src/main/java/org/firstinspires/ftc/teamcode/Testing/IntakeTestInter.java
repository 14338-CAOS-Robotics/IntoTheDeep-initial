package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls;
import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

public class IntakeTestInter extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.setOpMode(this);
        Intake intake = new Intake();
        intake.jointToTransfer();
        Controller controller = new Controller(gamepad1);
        intake.test();

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            Controller.update();
            intake.updateCoefficients();
            intake.testUpdate();
        }
    }
}
