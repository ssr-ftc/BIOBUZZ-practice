package org.firstinspires.ftc.teamcode.easyatl;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.easyatl.FieldPose;

/**
 * Writes an EasyATL field pose into the Pedro follower.
 * Kept as a TeamCode class so it compiles even when the AAR has no {@code PoseCorrector}.
 */
public final class PedroPoseCorrector {
    private final Follower follower;

    public PedroPoseCorrector(Follower follower) {
        if (follower == null) throw new IllegalArgumentException("follower cannot be null");
        this.follower = follower;
    }

    public void apply(FieldPose vision) {
        if (vision == null) return;
        follower.setPose(new Pose(vision.x, vision.y, vision.heading));
    }
}
