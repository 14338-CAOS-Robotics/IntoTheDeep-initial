package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.GamepadControls.JoystickControls;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

public class FieldCentricDriveOp extends LinearOpMode {

    Controller gp1;
    Controller gp2;

    @Override
    public void runOpMode() throws InterruptedException {
        gp1 = new Controller(gamepad1);
        gp2 = new Controller(gamepad2);
        Robot robot = new Robot();
        robot.setController1(gp1);
        robot.setController2(gp2);
        waitForStart();
        while (opModeIsActive()) {
            robot.update();

            robot.setFieldDrivePower(
                    gp1.get(JoystickControls.Input.LEFT, JoystickControls.Value.INVERT_X),
                    gp1.get(JoystickControls.Input.LEFT, JoystickControls.Value.Y),
                    gp1.get(JoystickControls.Input.RIGHT, JoystickControls.Value.X)
            );
            if (gp1.get(JoystickControls.Input.LEFT, JoystickControls.Value.INVERT_X) == 0 &&
            gp1.get(JoystickControls.Input.LEFT, JoystickControls.Value.Y) == 0 && gp1.get(JoystickControls.Input.RIGHT, JoystickControls.Value.X) == 0) {
                robot.setRobotDrivePower(
                        gp2.get(JoystickControls.Input.LEFT, JoystickControls.Value.INVERT_X),
                        gp2.get(JoystickControls.Input.LEFT, JoystickControls.Value.Y),
                        gp2.get(JoystickControls.Input.RIGHT, JoystickControls.Value.X)
                );
            }
        }
    }
}
