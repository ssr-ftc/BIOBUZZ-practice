package org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz;

/**
 * Whether the CELL's bottom-face AprilTag cluster is right-side up in the
 * camera frame. FIRST's BIOBUZZ tech tip uses {@code ftcPose.roll} for this:
 * a right-side-up cluster means that CELL is the one facing upward.
 */
public enum CellOrientation {
    FACING_UP,
    FACING_DOWN,
    UNKNOWN
}
