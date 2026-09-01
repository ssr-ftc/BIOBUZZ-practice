package org.firstinspires.ftc.teamcode.OFSB1.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.OFSB1.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * TeleOp with AprilTag assistance for OFSB1.
 *
 * Driving (same layout as OFSB1 TeleOp):
 *   - Left stick Y:  forward/backward
 *   - L2 / R2:       strafe left / right (analog)
 *   - Right stick X: turn (DISABLED while face-tag mode is on)
 *   - CROSS (hold):  slow mode - all driver inputs scaled to 1/2 speed
 *
 * AprilTag buttons (gamepad1):
 *   - TRIANGLE: toggle "face the tag" mode. On press, the tag is detected
 *     ONCE and its position is converted into a fixed FIELD coordinate
 *     using PedroPathing's localization. Odometry keeps the robot turned
 *     toward that stored point while you drive forward/strafe, constantly
 *     correcting; once within RELOCALIZE_DISTANCE_INCHES the camera
 *     refreshes the point whenever the tag is visible, so up-close facing
 *     stays dead-on. The right stick is disabled until you press TRIANGLE
 *     again to exit.
 *   - SQUARE: "park at the tag", in three phases:
 *       1. Localize ONCE from the press and drive a path toward the spot
 *          TARGET_DISTANCE_INCHES in front of the tag face (front bumper,
 *          perpendicular approach).
 *       2. Within RELOCALIZE_DISTANCE_INCHES: take ONE fresh detection of
 *          the same tag and rebuild the remaining path, correcting the
 *          long-range reading's error.
 *       3. On arrival at the standoff: visually CENTER the tag using the
 *          live camera bearing, turning until it sits in the middle of
 *          the view, then hand control back. Press again for a fresh run.
 *   - CIRCLE: cancel an in-progress park and return to driving.
 */
@TeleOp(name = "AprilTag Alignment", group = "OFSB1")
public class AprilTagAlignment extends OpMode {

    // How far the FRONT of the robot stops from the tag face (inches).
    private static final double TARGET_DISTANCE_INCHES = 5.0;

    // ---- Camera mounting geometry (inches), same as OFSB1Auto ----
    // Robot is 14.5 long x 13.5 wide; camera sits 8 from the back and 7
    // from the left. Pedro poses track ROBOT CENTER; AprilTag poses are
    // relative to the CAMERA LENS.
    private static final double CAMERA_FORWARD_OF_CENTER = 0.75;
    private static final double CAMERA_RIGHT_OF_CENTER = 0.25;
    private static final double CENTER_TO_FRONT = 7.25;

    // ---- Control tuning ----
    // Face-tag mode: turn power per RADIAN of heading error toward the
    // stored tag point. ~10 degrees of error -> ~0.35 power at 2.0.
    private static final double FACE_TAG_KP = 2.0;
    private static final double FACE_TAG_MAX_TURN = 0.5;
    // CROSS held = all driver inputs multiplied by this.
    private static final double SLOW_MODE_SCALE = 0.5;

    // Within this distance of the tag, trust the CAMERA over odometry: a
    // detection taken from far away (especially off to one side) bakes in
    // enough range/bearing error that the robot ends up aimed in front of
    // the tag instead of at it. Up close the camera is much more accurate,
    // so we re-localize with a fresh detection.
    private static final double RELOCALIZE_DISTANCE_INCHES = 18.0;

    // Final centering (after the park path arrives at the 5in standoff):
    // live camera bearing turns the robot until the tag is centered in the
    // camera view. Done when within this tolerance.
    private static final double CENTER_TOLERANCE_DEG = 1.5;
    private static final double CENTER_KP = 2.0; // turn power per radian of bearing
    private static final double CENTER_MAX_TURN = 0.4;
    // If the tag stays invisible this many consecutive loops during
    // centering (~1s), give up and hand control back rather than freezing.
    private static final int CENTER_MAX_MISSED_LOOPS = 50;
    // Hard cap on the centering phase: no matter what, autocorrect ends
    // and the driver gets control back after this many seconds.
    private static final double CENTER_TIMEOUT_SECONDS = 1.5;
    // Hard cap on the drive-to-tag phase, so a stalled path can never hold
    // the controls hostage.
    private static final double GOTO_TIMEOUT_SECONDS = 7.0;

    // Face-tag mode stops correcting heading when the robot is this close
    // to the stored point: aiming at a point nearly underneath the robot is
    // unstable (driving an inch past it flips the target heading 180
    // degrees, which made the robot spin in place).
    private static final double FACE_TAG_MIN_DISTANCE_INCHES = 10.0;

    private Follower follower;
    private AprilTagWebcam webcam;

    private enum Mode { DRIVE, GOTO_TAG, CENTER_TAG }
    private Mode mode = Mode.DRIVE;

    // Face-tag mode: the tag's position frozen into FIELD coordinates at
    // the moment triangle was pressed. Odometry does the rest, except
    // within RELOCALIZE_DISTANCE_INCHES where the camera refreshes the
    // stored point whenever the same tag is visible.
    private boolean faceTagEnabled = false;
    private double faceTagFieldX = 0;
    private double faceTagFieldY = 0;
    private int faceTagId = -1;

    // Square-button park: the tag's field position and id from the initial
    // detection, plus whether the close-range re-localization has run yet
    // (once per press).
    private double goToTagFieldX = 0;
    private double goToTagFieldY = 0;
    private int goToTagId = -1;
    private boolean relocalized = false;
    private int centerMissedLoops = 0;
    private double centerStartTime = 0;
    private double goToStartTime = 0;



    // Rising-edge detection so each button press acts exactly once.
    private boolean trianglePrev = false;
    private boolean squarePrev = false;
    private boolean circlePrev = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        webcam = new AprilTagWebcam();
        webcam.init(hardwareMap, telemetry);

        telemetry.addData("Status", "AprilTag Alignment Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        webcam.update();
        AprilTagDetection tag = getBestTag();
        telemetry.addData("Tag Visible", tag != null);
        webcam.displayDetectionTelemetry(tag);
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        webcam.update();

        boolean trianglePressed = gamepad1.triangle && !trianglePrev;
        boolean squarePressed = gamepad1.square && !squarePrev;
        boolean circlePressed = gamepad1.circle && !circlePrev;
        trianglePrev = gamepad1.triangle;
        squarePrev = gamepad1.square;
        circlePrev = gamepad1.circle;

        switch (mode) {
            case DRIVE:
                handleDrive(trianglePressed, squarePressed);
                break;

            case GOTO_TAG:
                if (circlePressed || getRuntime() - goToStartTime > GOTO_TIMEOUT_SECONDS) {
                    cancelToDrive();
                    break;
                }

                // Close enough that the camera beats odometry: take ONE
                // fresh detection of the SAME tag and rebuild the rest of
                // the path from it for an accurate final approach. This is
                // opportunistic - it must NOT block the arrival check
                // below, or an unseen tag locks the robot in this state
                // forever once the path finishes.
                if (!relocalized && nearTag(goToTagFieldX, goToTagFieldY)) {
                    AprilTagDetection fresh = webcam.getTagBySpecificId(goToTagId);
                    if (fresh != null && fresh.metadata != null && fresh.ftcPose != null) {
                        relocalized = true;
                        buildPathToTag(fresh);
                        break; // new path just started - check arrival next loop
                    }
                }

                if (!follower.isBusy()) {
                    // Arrived at the 5in standoff - finish by visually
                    // centering the tag in the camera.
                    follower.startTeleopDrive();
                    centerMissedLoops = 0;
                    centerStartTime = getRuntime();
                    mode = Mode.CENTER_TAG;
                }
                break;

            case CENTER_TAG:
                if (circlePressed) {
                    cancelToDrive();
                } else {
                    handleCenterTag();
                }
                break;
        }

        // ---- Telemetry ----
        telemetry.addData("Mode", mode);
        if (mode == Mode.GOTO_TAG) {
            telemetry.addData("Relocalized", relocalized ? "yes (camera-corrected)" : "not yet");
        }
        telemetry.addData("Slow Mode (cross)", gamepad1.cross ? "ON (1/2 speed)" : "off");
        if (faceTagEnabled) {
            telemetry.addData("Face Tag", "ON - locked on field point (%.1f, %.1f), right stick disabled",
                    faceTagFieldX, faceTagFieldY);
        } else {
            telemetry.addData("Face Tag", "off");
        }
        AprilTagDetection tag = getBestTag();
        telemetry.addData("Tag Visible", tag != null);
        webcam.displayDetectionTelemetry(tag);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private void handleDrive(boolean trianglePressed, boolean squarePressed) {
        AprilTagDetection tag = getBestTag();

        if (trianglePressed) {
            if (faceTagEnabled) {
                faceTagEnabled = false;
            } else if (tag != null) {
                // ONE camera detection: freeze the tag's location in field
                // coordinates. Odometry takes over from here (with camera
                // refreshes once we're close - see below).
                double[] tagField = tagFieldPosition(tag, follower.getPose());
                faceTagFieldX = tagField[0];
                faceTagFieldY = tagField[1];
                faceTagId = tag.id;
                faceTagEnabled = true;
            } else {
                telemetry.addLine("TRIANGLE: no tag visible - face mode not engaged");
            }
        }

        if (squarePressed) {
            if (tag != null) {
                relocalized = false; // fresh press = allow one re-localization
                goToStartTime = getRuntime();
                buildPathToTag(tag);
                return;
            } else {
                telemetry.addLine("SQUARE: no tag visible - try again");
            }
        }

        double scale = gamepad1.cross ? SLOW_MODE_SCALE : 1.0;
        double forward = -gamepad1.left_stick_y * scale;
        double strafe = (gamepad1.left_trigger - gamepad1.right_trigger) * scale;

        double turn;
        if (faceTagEnabled) {
            // Close-range accuracy fix: once within RELOCALIZE_DISTANCE of
            // the tag, refresh the stored field point from the camera
            // whenever the same tag is visible. Far away (or tag hidden),
            // the odometry snapshot keeps working like before.
            AprilTagDetection fresh = webcam.getTagBySpecificId(faceTagId);
            if (fresh != null && fresh.metadata != null && fresh.ftcPose != null
                    && fresh.ftcPose.range <= RELOCALIZE_DISTANCE_INCHES) {
                double[] tagField = tagFieldPosition(fresh, follower.getPose());
                faceTagFieldX = tagField[0];
                faceTagFieldY = tagField[1];
            }

            // Right stick disabled: heading is always corrected toward the
            // stored tag point, no matter how the robot translates. The
            // correction is NOT slowed by slow mode, so facing stays tight.
            Pose pose = follower.getPose();
            double distanceToPoint = Math.hypot(faceTagFieldX - pose.getX(),
                    faceTagFieldY - pose.getY());
            if (distanceToPoint >= FACE_TAG_MIN_DISTANCE_INCHES) {
                double desiredHeading = Math.atan2(faceTagFieldY - pose.getY(),
                        faceTagFieldX - pose.getX());
                double error = normalizeAngle(desiredHeading - pose.getHeading());
                turn = clamp(FACE_TAG_KP * error, FACE_TAG_MAX_TURN);
            } else {
                // Too close to aim at the point meaningfully - hold the
                // current heading instead of spinning after an unstable
                // target. Back away and correction resumes.
                turn = 0;
            }
        } else {
            turn = -gamepad1.right_stick_x * scale;
        }

        follower.setTeleOpDrive(forward, strafe, turn, true);
    }


    /**
     * Final phase of the square-button park: turn in place using the LIVE
     * camera bearing until the tag sits centered in the view. This wipes
     * out any residual heading error left over from odometry - the camera
     * itself is the judge of "centered".
     */
    private void handleCenterTag() {
        // Time's up - autocorrect never runs longer than the timeout,
        // centered or not.
        if (getRuntime() - centerStartTime > CENTER_TIMEOUT_SECONDS) {
            cancelToDrive();
            return;
        }

        AprilTagDetection fresh = webcam.getTagBySpecificId(goToTagId);

        if (fresh == null || fresh.metadata == null || fresh.ftcPose == null) {
            // Tag not visible this loop - hold position briefly; give up
            // only if it stays gone (e.g. someone stepped in front).
            centerMissedLoops++;
            follower.setTeleOpDrive(0, 0, 0, true);
            if (centerMissedLoops > CENTER_MAX_MISSED_LOOPS) {
                cancelToDrive();
            }
            return;
        }
        centerMissedLoops = 0;

        double bearingDeg = fresh.ftcPose.bearing;
        if (Math.abs(bearingDeg) <= CENTER_TOLERANCE_DEG) {
            // Tag centered - done, hand control back to the driver.
            cancelToDrive();
            return;
        }

        // Positive bearing = tag is left of center = turn CCW (positive).
        double turn = clamp(CENTER_KP * Math.toRadians(bearingDeg), CENTER_MAX_TURN);
        follower.setTeleOpDrive(0, 0, turn, true);
    }

    private void cancelToDrive() {
        follower.startTeleopDrive();
        follower.setTeleOpDrive(0, 0, 0, true);
        mode = Mode.DRIVE;
    }

    /** Closest tag with usable pose data, or null if none visible. */
    private AprilTagDetection getBestTag() {
        List<AprilTagDetection> tags = webcam.getDetectedTags();
        AprilTagDetection best = null;
        for (AprilTagDetection t : tags) {
            if (t.metadata == null || t.ftcPose == null) continue;
            if (best == null || t.ftcPose.range < best.ftcPose.range) best = t;
        }
        return best;
    }

    /**
     * One-shot: from a single detection, drive to the spot
     * TARGET_DISTANCE_INCHES in front of the tag FACE (perpendicular
     * approach), ending aimed straight at the tag.
     *
     * All math is done by converting the camera reading into FIELD
     * coordinates with a proper rotation by the robot's current heading -
     * valid for any heading, unlike the old mirrored-frame shortcut (which
     * sent the robot "back and left" when the heading wasn't zero).
     */
    private void buildPathToTag(AprilTagDetection tag) {
        Pose robotPose = follower.getPose();

        // Tag position relative to ROBOT CENTER (robot frame).
        double tagRight = tag.ftcPose.x + CAMERA_RIGHT_OF_CENTER;
        double tagForward = tag.ftcPose.y + CAMERA_FORWARD_OF_CENTER;

        // Tag position in FIELD coordinates. Remember it (and which tag it
        // was) so GOTO_TAG can re-localize on the same tag when close.
        double[] tagField = robotToField(tagRight, tagForward, robotPose, true);
        goToTagFieldX = tagField[0];
        goToTagFieldY = tagField[1];
        goToTagId = tag.id;

        // Unit vector pointing OUT of the tag face, robot frame. With
        // yaw = 0 the tag faces straight back at us: (right, forward) =
        // (0, -1). A rotated tag swings that normal by its yaw.
        // NOTE: if the robot parks on the wrong side of an ANGLED tag,
        // flip the sign of nRight - yaw sign conventions vary.
        double yaw = Math.toRadians(tag.ftcPose.yaw);
        double nRight = Math.sin(yaw);
        double nForward = -Math.cos(yaw);
        double[] nField = robotToField(nRight, nForward, robotPose, false);

        // Robot CENTER parks (front gap + center-to-front) out from the
        // tag along its normal -> the FRONT ends 5in from the tag,
        // perpendicular to its face.
        double standoff = TARGET_DISTANCE_INCHES + CENTER_TO_FRONT;
        double parkX = tagField[0] + standoff * nField[0];
        double parkY = tagField[1] + standoff * nField[1];

        // End heading: from the parking spot straight INTO the tag face.
        double faceFieldHeading = Math.atan2(tagField[1] - parkY, tagField[0] - parkX);

        Pose targetPose = new Pose(parkX, parkY, faceFieldHeading);
        Path pathToTag = new Path(new BezierLine(robotPose, targetPose));
        pathToTag.setLinearHeadingInterpolation(robotPose.getHeading(), faceFieldHeading);
        follower.followPath(pathToTag);
        mode = Mode.GOTO_TAG;
    }

    /** True when the robot center is within re-localization range of a field point. */
    private boolean nearTag(double fieldX, double fieldY) {
        Pose pose = follower.getPose();
        return Math.hypot(fieldX - pose.getX(), fieldY - pose.getY()) <= RELOCALIZE_DISTANCE_INCHES;
    }

    /** Tag's field-frame position from a detection and the current pose. */
    private double[] tagFieldPosition(AprilTagDetection tag, Pose robotPose) {
        double tagRight = tag.ftcPose.x + CAMERA_RIGHT_OF_CENTER;
        double tagForward = tag.ftcPose.y + CAMERA_FORWARD_OF_CENTER;
        return robotToField(tagRight, tagForward, robotPose, true);
    }

    /**
     * Converts robot-frame (right, forward) into field-frame {x, y}.
     * Pedro's field frame: heading is CCW-positive, and at heading 0 the
     * robot's forward axis is +x with its left side toward +y. Therefore:
     *   field = pose + forward * (cos h, sin h) + right * (sin h, -cos h)
     * Pass translate=false to rotate a direction vector without adding the
     * robot's position (for the tag normal).
     */
    private static double[] robotToField(double right, double forward, Pose pose, boolean translate) {
        double h = pose.getHeading();
        double x = forward * Math.cos(h) + right * Math.sin(h);
        double y = forward * Math.sin(h) - right * Math.cos(h);
        if (translate) {
            x += pose.getX();
            y += pose.getY();
        }
        return new double[] { x, y };
    }

    /** Wraps an angle to [-PI, PI]. */
    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private static double clamp(double value, double max) {
        return Math.max(-max, Math.min(max, value));
    }

    @Override
    public void stop() {
        webcam.stop();
    }
}
