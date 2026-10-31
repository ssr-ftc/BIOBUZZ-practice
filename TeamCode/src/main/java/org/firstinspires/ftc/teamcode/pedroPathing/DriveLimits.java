package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;

/** Maps the old follower.setMaxPower calls onto Pedro 3 path speed. */
public final class DriveLimits {
    private DriveLimits() {}

    public static void setMaxPower(Follower follower, double power) {
        ((Foresight) follower.algorithm()).config.maxPathSpeed.set(power);
    }
}
