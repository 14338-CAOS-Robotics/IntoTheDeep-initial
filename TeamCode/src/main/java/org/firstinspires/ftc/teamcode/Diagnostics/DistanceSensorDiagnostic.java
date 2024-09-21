package org.firstinspires.ftc.teamcode.Diagnostics;

import static org.firstinspires.ftc.teamcode.DashConstants.DistanceSensorDiagnostic.colorSensorID;
import static org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls.Input.TOUCHPAD;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Color_Sensor;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Distance_Sensor;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

@Disabled
@TeleOp(name = "DistanceSensorDiagnostic TeleOp", group="Diagnostic")
public class DistanceSensorDiagnostic extends LinearOpMode {
    private Controller controller;
    private Distance_Sensor distanceSensor;
    private String sensor_id = colorSensorID;

    public void initialize() {
        OpModeUtils.setOpMode(this);
        controller = new Controller(gamepad1);


        distanceSensor = new Distance_Sensor(sensor_id);

        OpModeUtils.multTelemetry.addData("Status", "Initialized");
        OpModeUtils.multTelemetry.addData("Start Keys", "Press [>] to begin");
        OpModeUtils.multTelemetry.addData("Shutdown Keys", "Press [RB] & [LB] simultaneously");
        OpModeUtils.multTelemetry.update();
    }

    public void shutdown() {
        OpModeUtils.multTelemetry.addData("Status", "Shutting Down");
        OpModeUtils.multTelemetry.update();
    }

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        while (opModeIsActive()) {
            Controller.update();
            distanceSensor.distanceUpdate();
            OpModeUtils.multTelemetry.addData("Millimeters", distanceSensor.getMM());
            OpModeUtils.multTelemetry.addData("Centimeters", distanceSensor.getCM());
            OpModeUtils.multTelemetry.addData("Meters", distanceSensor.getMeter());
            OpModeUtils.multTelemetry.update();


            // S H U T D O W N     S E Q U E N C E

            if (controller.get(TOUCHPAD, DOWN)) {
                shutdown();
                break;
            }
        }
    }
}
