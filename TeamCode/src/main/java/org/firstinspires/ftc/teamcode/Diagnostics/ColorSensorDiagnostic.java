package org.firstinspires.ftc.teamcode.Diagnostics;

import static org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls.Input.TOUCHPAD;
import static org.firstinspires.ftc.teamcode.DashConstants.ColorSensorDiagnostic.colorSensorID;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Color_Sensor;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

@Disabled
@TeleOp(name = "ColorSensorDiagnostic TeleOp", group="Diagnostic")
public class ColorSensorDiagnostic extends LinearOpMode {

    private Controller controller;

    private Color_Sensor colorSensor;
    private String sensor_id = colorSensorID;


    public void initialize() {
        OpModeUtils.setOpMode(this);
        controller = new Controller(gamepad1);


        colorSensor = new Color_Sensor();
        colorSensor.init(sensor_id);

        OpModeUtils.multTelemetry.addData("Status", "Initialized");
        OpModeUtils.multTelemetry.addData("Start Keys", "Press [>] to begin");
        OpModeUtils.multTelemetry.addData("Shutdown Keys", "Press [RB] & [LB] simultaneously");
        OpModeUtils.multTelemetry.update();
    }

    public void shutdown(){
        OpModeUtils.multTelemetry.addData("Status", "Shutting Down");
        OpModeUtils.multTelemetry.update();
    }


    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        while (opModeIsActive()) {
            Controller.update();

            OpModeUtils.multTelemetry.addData("Red", colorSensor.updateRed());
            OpModeUtils.multTelemetry.addData("Green", colorSensor.updateGreen());
            OpModeUtils.multTelemetry.addData("Blue", colorSensor.updateBlue());
            OpModeUtils.multTelemetry.addData("Alpha", colorSensor.colorSensor.alpha());
            OpModeUtils.multTelemetry.update();



            // S H U T D O W N     S E Q U E N C E

            if (controller.get(TOUCHPAD, DOWN)){
                shutdown();
                break;
            }
        }
    }
}


