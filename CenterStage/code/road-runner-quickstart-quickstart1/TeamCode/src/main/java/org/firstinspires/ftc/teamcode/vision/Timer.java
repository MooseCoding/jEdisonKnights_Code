package org.firstinspires.ftc.teamcode.vision;

public class Timer {
    private long sTime;

    public Timer() {
        sTime =  System.nanoTime();
    }
    public double getTimeSys() {
        return (System.nanoTime() - sTime)*(Math.pow(10, -9));
    }
}
