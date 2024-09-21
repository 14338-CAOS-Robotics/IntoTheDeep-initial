package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

public class AbsoluteEncoder {

    private AnalogInput encoder;
    private double lastPos;
    private double lastTime;

    public AbsoluteEncoder(String hardwareID){
        encoder = OpModeUtils.hardwareMap.get(AnalogInput.class, hardwareID);
    }

    public double getAngle() {
        return MathUtils.linearConversion(encoder.getVoltage() * 72, 1.3, 348, 0, 360);
    }

    public double getVelocity(){
        double retVal = (getAngle() - lastPos) / ((System.currentTimeMillis() / 1000.0) - lastTime);
        lastTime = System.currentTimeMillis() / 1000.0;
        lastPos = getAngle();
        return retVal;
    }
}
