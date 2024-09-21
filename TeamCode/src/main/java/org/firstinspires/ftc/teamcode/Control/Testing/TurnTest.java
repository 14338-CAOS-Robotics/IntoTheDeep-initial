package org.firstinspires.ftc.teamcode.Control.Testing;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Localization.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;

/**
 * OpMode to get the LATERAL_DISTANCE variable (distance between two horizontal wheels) all tuned up
 */
@TeleOp
@Config
public class TurnTest extends LinearOpMode {

    public static double startLateralDistance = 9.43015748;

    @Override
    public void runOpMode() throws InterruptedException {
        setOpMode(this);
        int counter = 0;
        CAOSHardware hardware = new CAOSHardware(hardwareMap);
        ThreeWheelOdometry odo1 = new ThreeWheelOdometry(0,0,0,hardware);

        double oldLateralDistance = startLateralDistance;
        double lateralDistance = oldLateralDistance;
        //heading1 < 180 degrees, heading2 > 180 degrees
        double heading = 3;
        double measured;

        odo1.setLateralDistance(lateralDistance);
        telemetry.addData("Instruction", "Turn the robot 180 degrees whenever you see the counter increase");
        telemetry.update();
        while (opModeIsActive()) {
            measured = odo1.getHeading();
            double newLateralDistance = (lateralDistance - oldLateralDistance)*(Math.PI - measured) / (measured - heading) + lateralDistance;
            oldLateralDistance = lateralDistance;
            lateralDistance = newLateralDistance;
            odo1.setLateralDistance(lateralDistance);
            odo1.setHeading(0);
            sleep(6000);



        }
    }
}
