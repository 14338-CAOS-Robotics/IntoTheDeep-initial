package org.firstinspires.ftc.teamcode.Utilities.Files.BlackBox;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="BlackBox TeleOp", group="Iterative Opmode")
public class BlackBoxTestingOp extends OpMode {

    private BlackBoxLogger logger;

    @Override
    public void init() {
        setOpMode(this);

        logger = new BlackBoxLogger();

        multTelemetry.addData("Status", "Initialized");
        multTelemetry.update();
    }


    @Override
    public void init_loop() {

    }


    @Override
    public void start() {

        multTelemetry.addData("Status", "Started");
        multTelemetry.update();
    }


    @Override
    public void loop() {

        logger.writeData();

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