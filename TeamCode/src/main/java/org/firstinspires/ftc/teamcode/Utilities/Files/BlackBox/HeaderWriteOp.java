package org.firstinspires.ftc.teamcode.Utilities.Files.BlackBox;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name="HeaderWriteOP", group="Iterative Opmode")
public class HeaderWriteOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private BlackBoxLogger blackBoxLogger;

    @Override
    public void init() {
        setOpMode(this);

        blackBoxLogger = new BlackBoxLogger();
        multTelemetry.addData("Status", "Initialized");
        multTelemetry.update();
    }


    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
        runtime.reset();
        blackBoxLogger.writeHeaders();


        multTelemetry.addData("Status", "Started");
        multTelemetry.update();
    }


    @Override
    public void loop() {

        multTelemetry.addData("Written!", "Success");
        multTelemetry.update();

    }

    @Override
    public void stop(){

    }




    //PUT ALL UPDATE METHODS HERE


    //Telemetry to be displayed during init_loop()

    private void initTelemetry(){
        multTelemetry.addData("Status", "InitLoop");
        multTelemetry.update();
    }

    //Telemetry to be displayed during loop()

    private void loopTelemetry(){
        multTelemetry.addData("Status", "TeleOp Running");
        multTelemetry.update();
    }
}