package org.firstinspires.ftc.teamcode.Utilities.Files.BlackBox;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static java.lang.Math.floor;
import static java.lang.Math.log10;
import static java.lang.Math.pow;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Utilities.Files.FileUtils.FileLogWriter;
import org.firstinspires.ftc.teamcode.Utilities.Files.Loggers.GridLoggerEx;
import org.firstinspires.ftc.teamcode.Utilities.revextensions2.ExpansionHubEx;

import java.util.ArrayList;
import java.util.List;

public class BlackBoxLogger {

    private final String FILENAME = "TeamCodeLog";

    private GridLoggerEx mainLogger;
    private FileLogWriter mainWriter;

    private ArrayList<ExpansionHubEx> hubs = new ArrayList<ExpansionHubEx>();
    private VoltageSensor batteryVoltage;
    private LoopTimer loopTimer;

    public BlackBoxLogger(){

        mainWriter = new FileLogWriter(FILENAME);
        mainLogger = new GridLoggerEx(mainWriter);

        List<LynxModule> baseHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule baseHub: baseHubs) {
            hubs.add(new ExpansionHubEx(baseHub));
        }
        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        mainLogger.addHeader("Time");
        //TODO ADD HEADER SUPPORT
        mainLogger.addHeader("HeaderID");
        mainLogger.addHeader("Hertz");
        mainLogger.addHeader("Battery");

        mainLogger.addHeader("XPos");
        mainLogger.addHeader("YPos");
        mainLogger.addHeader("GyroReading");
        mainLogger.addHeader("ThumbstickRot");
        mainLogger.addHeader("RotInput");
        mainLogger.addHeader("RawIMU");
        for (ExpansionHubEx hub: hubs) {
            String deviceName = hub.getStandardModule().getSerialNumber().getString();
            mainLogger.addHeader("ServoDraw" + deviceName);
            mainLogger.addHeader("SensorDraw" + deviceName);
            mainLogger.addHeader("Motor0Hub" + deviceName);
            mainLogger.addHeader("Motor1Hub" + deviceName);
            mainLogger.addHeader("Motor2Hub" + deviceName);
            mainLogger.addHeader("Motor3Hub" + deviceName);
            mainLogger.addHeader("12v" + deviceName);
            mainLogger.addHeader("5v" + deviceName);
        }
        loopTimer = new LoopTimer();

        mainLogger.writeHeaders();
    }

    public void log(String column, double value, int decimals){
        /*
        String precision = new String(new char[decimals]).replace("\0", "#");
        DecimalFormat numFormat = new DecimalFormat("###." + precision);
        String truncatedData = numFormat.format(value);
         */
        double truncatedValue = floor(value * pow(10, decimals)) / pow(10, decimals);
        mainLogger.add(column, truncatedValue);
    }

    //TODO: This is currently kinda slow, so if this could do something like how
    // decimals are handled for doubles that would be ideal
    public void log(String column, int value, int decimals){
        /*
        String precision = new String(new char[decimals]).replace("\0", "#");
        DecimalFormat numFormat = new DecimalFormat("0." + precision + "E0");
        String truncatedData = numFormat.format(value);
         */
        double magnitude = (int) floor(log10(value));
        multTelemetry.addData("m", magnitude);
        double significand = value / pow(10, magnitude);
        multTelemetry.addData("s", significand);
        double truncatedSignificand = floor(value * pow(10, decimals)) / pow(10, decimals);
        multTelemetry.addData("ts", truncatedSignificand);
        double truncatedValue = truncatedSignificand * pow(10, magnitude);
        multTelemetry.addData("tv", truncatedValue);
        mainLogger.add(column, truncatedValue);
    }

    public void log(String column, double value){
        mainLogger.add(column, value);
    }

    public void log(String column, String data){
        mainLogger.add(column, data);
    }

    public void writeData(){
        internalDataUpdate();

        ArrayList<String> failedColumns = mainLogger.writeRow();
        multTelemetry.addData("BlackBoxErrs:", failedColumns);
    }

    private void internalDataUpdate(){
        loopTimer.update();

        log("Time", System.currentTimeMillis());
        log("Hertz", loopTimer.getHertz(), 2);
        log("Battery", batteryVoltage.getVoltage(), 3);

        for (ExpansionHubEx hub: hubs) {
            String deviceName = hub.getStandardModule().getModuleSerialNumber().getString();
            log("ServoDraw" + deviceName, hub.getServoBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS), 2);
            log("SensorDraw" + deviceName, hub.getI2cBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS), 2);
            log("Motor0Hub" + deviceName, hub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, 0), 1);
            log("Motor1Hub" + deviceName, hub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, 1), 1);
            log("Motor2Hub" + deviceName, hub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, 2), 1);
            log("Motor3Hub" + deviceName, hub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, 3), 1);
            log("12v" + deviceName, hub.read12vMonitor(ExpansionHubEx.VoltageUnits.MILLIVOLTS), 5);
            log("5v" + deviceName, hub.read5vMonitor(ExpansionHubEx.VoltageUnits.MILLIVOLTS), 5);
        }
    }

    public void writeHeaders(){
        mainLogger.writeHeaders();
    }

    public void deleteLog(){
        mainWriter.deleteFile();
    }

}
