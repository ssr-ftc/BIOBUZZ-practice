package org.firstinspires.ftc.teamcode.OFSB1.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.OFSB1.Constants;
import org.firstinspires.ftc.teamcode.OFSB1.Subsystems.OFSB1Subsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;
import java.util.List;

/**
 * TeleOp that drives like OFSB1 TeleOp and relocalizes off an AprilTag
 * taped at field (8, 8) inches, facing 45 degrees into the field.
 *
 * Driving (gamepad1):
 *   - Left stick Y:  forward / backward
 *   - L2 / R2:       strafe left / right (analog)
 *   - Right stick X: turn
 *
 * TRIANGLE: hold still. Collects several camera frames, median-filters
 * them, and snaps Pedro to the result. Tag is (8, 8), face into the
 * field at 45 deg, so square-on robot heading is 225 deg.
 */
@TeleOp(name = "AprilTag Localize", group = "OFSB1")
public class AprilTagLocalize extends OpMode {

    private static final double TAG_FIELD_X = 8.0;
    private static final double TAG_FIELD_Y = 8.0;

    // Robot heading when the *camera* looks square at the tag (yaw = 0).
    // Tag faces 45 deg into the field; camera looks back at it from 225.
    private static final double TAG_FACE_HEADING_RAD = Math.PI + Math.PI / 4.0;

    // Camera on the robot, relative to robot CENTER (inches).
    // Robot is 14.5 long x 13.5 wide; camera 8 from the back, 7 from the left.
    private static final double CAMERA_FORWARD_OF_CENTER = 0.75;
    private static final double CAMERA_RIGHT_OF_CENTER = 0.25;

    // Camera yaw relative to the robot, CCW-positive (degrees).
    // Negative = lens points slightly right of robot forward.
    // Tuned from (47, 46) reading as (49, 43): ~4 deg clockwise mount/yaw bias.
    // If X/Y are still rotated around the tag, change this by ~1 deg at a time.
    private static final double CAMERA_YAW_DEG = -4.0;

    // More pixels on the tag = tighter pose. 1 is slower than Alignment's 2.
    private static final float LOCALIZE_DECIMATION = 1;

    private static final int BURST_SAMPLES = 8;
    private static final double BURST_MIN_DT_SEC = 0.04;
    private static final double BURST_TIMEOUT_SEC = 1.5;

    private Follower follower;
    private OFSB1Subsystem robot;
    private AprilTagWebcam webcam;

    private boolean trianglePrev = false;
    private String lastLocalizeStatus = "not yet";

    private boolean bursting = false;
    private int burstCount = 0;
    private double burstStartTime = 0;
    private double lastSampleTime = 0;
    private final double[] burstX = new double[BURST_SAMPLES];
    private final double[] burstY = new double[BURST_SAMPLES];
    private final double[] burstH = new double[BURST_SAMPLES];

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        robot = new OFSB1Subsystem(hardwareMap);

        webcam = new AprilTagWebcam();
        webcam.init(hardwareMap, telemetry);
        webcam.setDecimation(LOCALIZE_DECIMATION);

        telemetry.addData("Status", "AprilTag Localize Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        webcam.update();
        AprilTagDetection tag = closestTag();
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
        trianglePrev = gamepad1.triangle;

        if (trianglePressed && !bursting) {
            startBurst();
        }

        if (bursting) {
            updateBurst();
        }

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_trigger - gamepad1.right_trigger;
        double turn = -gamepad1.right_stick_x;
        follower.setTeleOpDrive(forward, strafe, turn, true);

        Pose pose = follower.getPose();
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Last localize", lastLocalizeStatus);
        if (bursting) {
            telemetry.addData("Localizing", "HOLD STILL  %d/%d", burstCount, BURST_SAMPLES);
        }

        AprilTagDetection tag = closestTag();
        telemetry.addData("Tag Visible", tag != null);
        webcam.displayDetectionTelemetry(tag);
        telemetry.update();
    }

    private void startBurst() {
        if (closestTag() == null) {
            lastLocalizeStatus = "TRIANGLE: no tag visible";
            return;
        }
        bursting = true;
        burstCount = 0;
        burstStartTime = getRuntime();
        lastSampleTime = -1;
        lastLocalizeStatus = "sampling...";
    }

    private void updateBurst() {
        if (getRuntime() - burstStartTime > BURST_TIMEOUT_SEC) {
            bursting = false;
            lastLocalizeStatus = burstCount == 0
                    ? "TRIANGLE: tag lost"
                    : "TRIANGLE: timeout, " + burstCount + " samples — try again still";
            return;
        }

        AprilTagDetection tag = closestTag();
        if (tag == null) {
            return;
        }

        double now = getRuntime();
        if (lastSampleTime >= 0 && now - lastSampleTime < BURST_MIN_DT_SEC) {
            return;
        }

        Pose sample = poseFromDetection(tag);
        burstX[burstCount] = sample.getX();
        burstY[burstCount] = sample.getY();
        burstH[burstCount] = sample.getHeading();
        burstCount++;
        lastSampleTime = now;

        if (burstCount >= BURST_SAMPLES) {
            bursting = false;
            double x = median(burstX, burstCount);
            double y = median(burstY, burstCount);
            double h = circularMean(burstH, burstCount);
            follower.setPose(new Pose(x, y, h));
            lastLocalizeStatus = String.format("tag %d -> (%.1f, %.1f) in, %.1f deg  [%d frm]",
                    tag.id, x, y, Math.toDegrees(h), burstCount);
        }
    }

    /**
     * Tag-in-camera -> robot-center-in-field, using the known tag pose
     * and camera extrinsics (translation + yaw).
     */
    private Pose poseFromDetection(AprilTagDetection tag) {
        double camYaw = Math.toRadians(CAMERA_YAW_DEG);
        double camRight = tag.ftcPose.x;
        double camForward = tag.ftcPose.y;

        // Rotate camera-frame (right, forward) into robot-frame, then
        // add the lens offset from robot center.
        double tagForward = camForward * Math.cos(camYaw) + camRight * Math.sin(camYaw)
                + CAMERA_FORWARD_OF_CENTER;
        double tagRight = -camForward * Math.sin(camYaw) + camRight * Math.cos(camYaw)
                + CAMERA_RIGHT_OF_CENTER;

        // Camera optical axis vs tag face; then undo camera yaw to get robot heading.
        double heading = TAG_FACE_HEADING_RAD - Math.toRadians(tag.ftcPose.yaw) - camYaw;

        double offsetX = tagForward * Math.cos(heading) + tagRight * Math.sin(heading);
        double offsetY = tagForward * Math.sin(heading) - tagRight * Math.cos(heading);
        return new Pose(TAG_FIELD_X - offsetX, TAG_FIELD_Y - offsetY, heading);
    }

    private AprilTagDetection closestTag() {
        List<AprilTagDetection> tags = webcam.getDetectedTags();
        AprilTagDetection best = null;
        for (AprilTagDetection t : tags) {
            if (t.metadata == null || t.ftcPose == null) continue;
            if (best == null || t.ftcPose.range < best.ftcPose.range) {
                best = t;
            }
        }
        return best;
    }

    private static double median(double[] values, int n) {
        double[] copy = Arrays.copyOf(values, n);
        Arrays.sort(copy);
        if ((n & 1) == 1) {
            return copy[n / 2];
        }
        return 0.5 * (copy[n / 2 - 1] + copy[n / 2]);
    }

    private static double circularMean(double[] angles, int n) {
        double s = 0;
        double c = 0;
        for (int i = 0; i < n; i++) {
            s += Math.sin(angles[i]);
            c += Math.cos(angles[i]);
        }
        return Math.atan2(s, c);
    }

    @Override
    public void stop() {
        robot.stopAll();
        webcam.stop();
    }
}
