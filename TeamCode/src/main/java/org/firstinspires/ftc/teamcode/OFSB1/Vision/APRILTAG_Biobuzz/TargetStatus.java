package org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz;

public enum TargetStatus {
    /** Nothing relevant is visible, and we have not locked a cell yet. */
    SEARCHING,
    /** A lock existed, then the timeout expired with nothing to replace it. */
    NO_TARGET,
    /** Last valid cell is being held across a short detection gap. */
    TEMPORARILY_LOST,
    /** An alliance cell is visible, but the pose is not good enough to use. */
    LOW_CONFIDENCE,
    /** An alliance cell is visible and should not be shot. See the reason. */
    TARGET_INVALID,
    /** Alliance cell, upward, with a usable pose. Shootable is a stricter flag. */
    TARGET_VALID
}
