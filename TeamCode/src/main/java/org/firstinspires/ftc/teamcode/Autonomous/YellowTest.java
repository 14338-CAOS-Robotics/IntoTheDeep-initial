package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

@Autonomous
public class YellowTest extends LinearOpMode {
    Movement movement;
    CAOSHardware hardware;
    Robot robot;
    //MultipleTelemetry telemetry;
    AutonomousUtils auto;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.setOpMode(this);
        hardware = new CAOSHardware(hardwareMap);
        movement = new Movement(0,0,0,hardware);
        robot = new Robot(false);
        auto = new AutonomousUtils(movement, robot, hardwareMap, this.telemetry);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            //boolean done = auto.goRelativeToAprilTag(5,12,0,0,1,0.1);
            //telemetry.addData("successful", done);
            //while (!movement.snapHeading(Math.PI, 0.1)) {}
            while (!isStopRequested()) {
                telemetry.addData("heading", movement.odo.getHeading());
                telemetry.addData("heading error", movement.catchJump(Math.PI, movement.odo.getHeading()));
                telemetry.addData("heading power", movement.getHeadingPower(movement.catchJump(Math.PI, movement.odo.getHeading())));
                telemetry.update();
                movement.odo.localize();
            }
            sleep(30000);
        }
    }
}
