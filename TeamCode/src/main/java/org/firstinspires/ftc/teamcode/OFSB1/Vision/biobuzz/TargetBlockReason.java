package org.firstinspires.ftc.teamcode.OFSB1.Vision.biobuzz;

/**
 * Why a visible cell is not a shot. Empty string when nothing is blocking.
 */
public enum TargetBlockReason {
    NONE(""),
    ALLIANCE_NOT_SELECTED("ALLIANCE NOT SELECTED"),
    CAMERA_NOT_READY("CAMERA NOT READY"),
    NO_RELEVANT_TAGS("NO RELEVANT TAGS VISIBLE"),
    OPPOSING_ALLIANCE_ONLY("OPPOSING ALLIANCE TAGS ONLY"),
    POSE_UNAVAILABLE("POSE UNAVAILABLE"),
    FACING_DOWN("CELL FACING DOWN"),
    ORIENTATION_UNKNOWN("ORIENTATION UNKNOWN"),
    LOW_CONFIDENCE("LOW CONFIDENCE"),
    NOT_ON_SHOOTING_SIDE("NOT ON SHOOTING SIDE"),
    BEARING_TOO_LARGE("NOT AIMED"),
    TOO_CLOSE("TOO CLOSE"),
    TOO_FAR("TOO FAR"),
    TEMPORARILY_LOST("TARGET LOST - HOLDING LAST");

    public final String telemetry;

    TargetBlockReason(String telemetry) {
        this.telemetry = telemetry;
    }
}
