package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.util.ElapsedTime;

/** Stand-in for the Pedro 2 timer. Pedro 3 no longer ships com.pedropathing.util.Timer. */
public class PathTimer {
    private final ElapsedTime elapsed = new ElapsedTime();

    public double getElapsedTimeSeconds() {
        return elapsed.seconds();
    }

    public void resetTimer() {
        elapsed.reset();
    }
}
