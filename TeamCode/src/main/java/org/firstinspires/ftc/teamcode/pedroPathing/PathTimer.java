package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.util.ElapsedTime;

/** Replaces the Pedro 2 timer. Pedro 3 does not include org.firstinspires.ftc.teamcode.pedroPathing.PathTimer. */
public class PathTimer {
    private final ElapsedTime elapsed = new ElapsedTime();

    public double getElapsedTimeSeconds() {
        return elapsed.seconds();
    }

    public void resetTimer() {
        elapsed.reset();
    }
}
