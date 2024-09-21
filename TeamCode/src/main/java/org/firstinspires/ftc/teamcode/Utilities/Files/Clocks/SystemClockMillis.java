package org.firstinspires.ftc.teamcode.Utilities.Files.Clocks;

public class SystemClockMillis implements Clock {
    private long startTime;
    public SystemClockMillis(){
        startTime = System.currentTimeMillis();
    }

    public long getCurrentTime(){
        return System.currentTimeMillis();
    }
    public double getTimePassed(){
        return (getCurrentTime() - startTime);
    }

}
