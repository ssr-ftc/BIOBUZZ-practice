package org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz;

/**
 * Every number that should be tuned on the real HIVE lives here.
 * Units are inches and degrees unless the name says otherwise.
 *
 * <p>SDK note: this project is FTC SDK 11.1. AprilTag Clusters (one detection
 * per CELL, pose aimed at the goal opening) arrive in SDK 12. Until then the
 * pose below is the fused center of the visible tags on the CELL's bottom
 * sticker, in the camera frame the SDK already uses:
 * X right, Y forward, Z up. Distance is that 3D translation, not tag pixels.
 */
public final class BioBuzzVisionConfig {

    private BioBuzzVisionConfig() {
    }

    /**
     * Black-square size of a BIOBUZZ AprilTag. Game manual section 9: tags
     * are 3.25 in square, family 36h11. If every distance reads consistently
     * long or short, this size does not match the printed black border.
     */
    public static double TAG_SIZE_INCHES = 3.25;

    /**
     * Webcam pose relative to the robot center. Same measurements already
     * used by AprilTag Alignment / AprilTag Localize (robot 14.5 x 13.5 in,
     * camera 8 in from the back and 7 in from the left).
     */
    public static double CAMERA_FORWARD_OF_CENTER_INCHES = 0.75;
    public static double CAMERA_RIGHT_OF_CENTER_INCHES = 0.25;

    /**
     * Lens direction relative to robot forward. Same signs EasyATL uses:
     * negative yaw = lens pointed right, negative pitch = tilted down.
     * The -4° yaw is the bias already tuned on this robot.
     * Change pitch here when the camera is tilted. Do not change it when
     * the robot merely drives to a new spot.
     */
    public static double CAMERA_YAW_RADIANS = Math.toRadians(-4.0);
    public static double CAMERA_PITCH_RADIANS = 0.0;

    /**
     * FTC field frame, inches, origin at the center of the field.
     * Heading 0 points along +X. Positive heading is counterclockwise.
     * On this field +X points toward the audience and +Y points from the
     * red wall toward the blue wall.
     *
     * <p>The game manual places the two cells on one HIVE 18.8 in apart, so
     * each cell sits 9.4 in from that hive's pivot along X. It does not
     * publish the red-hive / blue-hive Y. Tape {@link #RED_HIVE_Y_INCHES}
     * and {@link #BLUE_HIVE_Y_INCHES} to the sticker center with the cell
     * in the up position you actually look at. The sticker swings when the
     * hive tips, and EasyATL treats these coordinates as fixed.
     */
    public static double CELL_OFFSET_FROM_PIVOT_INCHES = 9.4;
    public static double RED_HIVE_Y_INCHES = -12.0;
    public static double BLUE_HIVE_Y_INCHES = 12.0;

    /** EasyATL acceptance filters. Wider than a goal-tag setup so a side view of the HIVE still localizes. */
    public static double FIELD_MAX_RANGE_INCHES = 140.0;
    public static double FIELD_MAX_BEARING_DEGREES = 75.0;
    public static double FIELD_MAX_TAG_YAW_DEGREES = 70.0;
    public static double FIELD_OUTLIER_DISTANCE_INCHES = 18.0;
    public static double FIELD_OUTLIER_HEADING_DEGREES = 35.0;
    public static double FIELD_SMOOTHING_ALPHA = 0.45;
    /** Caps how far one frame may move the reported field pose. Stops a bad detect from teleporting it. */
    public static double FIELD_MAX_STEP_INCHES = 24.0;
    public static double FIELD_MAX_STEP_DEGREES = 45.0;

    /**
     * Extra inches added along camera-forward before distance is computed.
     * Leave at 0 until you measure how far the goal opening sits in front
     * of the tag sticker. Horizontal distance is otherwise the distance to
     * the tag cluster itself.
     */
    public static double OPENING_FORWARD_OF_TAGS_INCHES = 0.0;

    /** Added to camera-frame Z. Does not change horizontal distance. */
    public static double OPENING_ABOVE_TAGS_INCHES = 0.0;

    /**
     * FIRST's published rule is |roll| &lt; 90°: the cluster is right-side up,
     * so that CELL is the one facing upward.
     *
     * <p>That test is the same everywhere on the audience side and everywhere
     * on the scoring side. The robot does not have to stand on one marked
     * spot. The camera does have to be looking at the HIVE so the tags are
     * in frame. A roll of exactly 90° is UNKNOWN, because that is the tip point.
     */
    public static double FACING_UP_MAX_ROLL_DEG = 90.0;
    public static double FACING_DOWN_MIN_ROLL_DEG = 90.0;

    /**
     * One tag is enough to call the cell up or down. Below this decision
     * margin the decode is too weak, so the cell stays not shootable.
     */
    public static double MIN_MARGIN_FOR_ORIENTATION = 20.0;

    /** Detections with more bit errors than this are dropped before grouping. */
    public static int MAX_HAMMING = 1;

    /**
     * Confidence from how much of the 4-tag cluster is in the pose.
     * Shootable only needs one tag above {@link #MIN_MARGIN_FOR_ORIENTATION}.
     * More tags raise this from LOW to MEDIUM or HIGH.
     */
    public static int MEDIUM_MIN_TAGS = 2;
    public static int HIGH_MIN_TAGS = 3;
    public static double MEDIUM_MIN_DECISION_MARGIN = 40.0;
    public static double HIGH_MIN_DECISION_MARGIN = 80.0;

    /** A new cell must beat the locked cell by this many score points to take over. */
    public static double TARGET_SWITCH_MARGIN = 80.0;

    /**
     * How long a valid cell may disappear before the lock is cleared.
     * Short on purpose: long enough to skip one bad frame, short enough
     * that a real loss does not keep aiming at empty air.
     */
    public static double LOST_TARGET_TIMEOUT_SECONDS = 0.30;

    /**
     * Bearing inside this band, with a shootable (facing-up) cell, is
     * READY_TO_SHOOT. Being outside this band does not make the cell
     * not shootable. It only means the robot is not aimed yet.
     */
    public static double READY_BEARING_DEG = 3.0;

    /** Turn power per radian of bearing while vision assist is aiming. */
    public static double AIM_KP = 2.0;
    public static double AIM_MAX_TURN = 0.45;

    /** Right-stick magnitude above this cancels the aim correction for that loop. */
    public static double DRIVER_TURN_OVERRIDE = 0.20;
}
