package org.firstinspires.ftc.teamcode;

public class RobotTimer {
    long startTime;
    long waitTime;
    public RobotTimer(long time) {
        waitTime = time;
    }
    public void start() {
        startTime = System.currentTimeMillis();
    }
    public boolean IsDone() {
        long elapsed = System.currentTimeMillis() - startTime;
        if (elapsed > waitTime) {
            return true;
        }
        return false;
    }
}
