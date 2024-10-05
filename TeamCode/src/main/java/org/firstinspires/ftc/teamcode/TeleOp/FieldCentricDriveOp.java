package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.GamepadControls.JoystickControls;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

/**
 * Controller 1 controls in field centric mode. Controller 2 in robot centric mode
 */
@TeleOp
public class FieldCentricDriveOp extends LinearOpMode {

    Controller gp1;
    Controller gp2;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.setOpMode(this);
        gp1 = new Controller(gamepad1);
        gp2 = new Controller(gamepad2);
        Robot robot = new Robot();
        robot.setController1(gp1);
        robot.setController2(gp2);
        waitForStart();

        while (opModeIsActive()) {
            robot.update();

            if (gp1.joyStickUsed(JoystickControls.Input.LEFT) || gp1.joyStickUsed(JoystickControls.Input.RIGHT)) {
                robot.setFieldDrivePower(
                        gp1.get(JoystickControls.Input.LEFT, JoystickControls.Value.INVERT_X),
                        gp1.get(JoystickControls.Input.LEFT, JoystickControls.Value.Y),
                        gp1.get(JoystickControls.Input.RIGHT, JoystickControls.Value.X)
                );
            } else {
                robot.setRobotDrivePower(
                        gp2.get(JoystickControls.Input.LEFT, JoystickControls.Value.Y),
                        gp2.get(JoystickControls.Input.LEFT, JoystickControls.Value.INVERT_X),
                        gp2.get(JoystickControls.Input.RIGHT, JoystickControls.Value.X)
                );
            }
            telemetry.addData("lefty", gp1.get(JoystickControls.Input.LEFT, JoystickControls.Value.Y));
            telemetry.addData("leftx", gp1.get(JoystickControls.Input.LEFT, JoystickControls.Value.X));
            telemetry.addData("rightx", gp1.get(JoystickControls.Input.RIGHT, JoystickControls.Value.X));
            telemetry.update();
        }
    }
}
