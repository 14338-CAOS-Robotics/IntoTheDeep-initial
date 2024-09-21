package org.firstinspires.ftc.teamcode.Utilities.Files.BlackBox;

public class LoopTimer {

    private long prevTime;

    private double hz = 0;
    private double nanos = 0;

    public LoopTimer(){
        prevTime = System.nanoTime();
    }

    public void reset(){
        prevTime = System.nanoTime();
    }

    public void update(){
        nanos = System.nanoTime() - prevTime;
        prevTime = System.nanoTime();

        hz = 1E+9 / nanos;
    }

    public double getHertz(){
        return hz;
    }

    public double getNanos(){
        return nanos;
    }

    public double getSeconds(){
        return nanos / 1E+9;
    }


}
