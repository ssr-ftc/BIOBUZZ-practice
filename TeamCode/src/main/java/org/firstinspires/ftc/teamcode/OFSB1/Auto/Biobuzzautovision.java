package org.firstinspires.ftc.teamcode.OFSB1.Auto;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.easyobjd.ClusterInfo;
import org.firstinspires.ftc.easyobjd.EasyOBJD;
import org.firstinspires.ftc.easyobjd.EasyOBJDConfig;
import org.firstinspires.ftc.easyobjd.EasyOBJDPipeline;
import org.firstinspires.ftc.easyobjd.OverlayMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OFSB1.Constants;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz.Alliance;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz.AllianceConfig;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

/**
 * BioBuzz autonomous.
 *
 * INIT:  Circle / B = RED.  X = BLUE (Square on PlayStation, X on Xbox;
 *        Cross / A also works for blue).
 * START: wait 3 s for the hive to tip -> turn 90 deg (RED = left, BLUE = right)
 *        -> search up to 3 s for clusters -> if none, turn 180 and search again
 *        -> drive to the largest cluster -> drive back to the starting pose.
 *
 * Driving works like your working "Follow Ball" OpMode: startTeleopDrive() once,
 * then every loop follower.update() + setTeleOpDrive(forward, strafe, turn, true)
 * with proportional power. The Pedro pose (odometry) is the error signal, so there
 * are no paths or turnTo() calls. Turn sign: positive turn power = counter-clockwise
 * (left), the same convention as your TeleOp (right stick X is negated there).
 */
@SuppressWarnings("unused")
@Autonomous(name = "BioBuzz Auto", group = "EasyOBJD")
public class Biobuzzautovision extends LinearOpMode {

    // ---------------- Camera (must match the Range Test you calibrated with) ----------------
    public static final String WEBCAM_NAME = "Webcam 1";
    public static final int STREAM_WIDTH = 640;
    public static final int STREAM_HEIGHT = 480;
    /**
     * IMPORTANT: use the SAME rotation you used when you ran the Range Test.
     * Your Follow Ball OpMode streams UPSIDE_DOWN. If the webcam is physically mounted
     * upside down, set this to UPSIDE_DOWN and re-run the Range Test that way, or every
     * distance will be wrong.
     */
    public static final OpenCvCameraRotation CAMERA_ROTATION = OpenCvCameraRotation.UPRIGHT;
    public static double CAMERA_HEIGHT_INCHES = 18.0;
    public static double CAMERA_TILT_DEGREES = 25.0;
    public static double HORIZONTAL_FOV_DEGREES = 70.4;
    public static double FOCAL_LENGTH_PIXELS_AT_640 = 0;
    public static double BALL_DIAMETER_INCHES = 2.8;

    // ---------------- Robot geometry (inches) ----------------
    /**
     * How far the lens is ahead of the robot's tracking center (the point Pedro reports
     * as the pose). Negative if the lens is behind center. MEASURE THIS.
     */
    public static double LENS_FORWARD_OF_CENTER_INCHES = 0.0;
    /**
     * How far from the cluster the robot CENTER stops. 0 = the tracking center ends up
     * on the ball. Set it to the center-to-front distance to stop with the front at the ball.
     */
    public static double STOP_SHORT_INCHES = 0.0;

    // ---------------- Auto tuning ----------------
    public static long HIVE_WAIT_MS = 3000;
    public static double TURN_DEGREES = 90.0;
    public static long SEARCH_MS = 3000;
    private static final int SETTLE_FRAMES = 5;

    // Drive gains (same starting values as your Follow Ball OpMode).
    private static final double FORWARD_KP = 0.035;   // power per inch
    private static final double TURN_KP = 0.02;       // power per degree
    private static final double MAX_DRIVE_POWER = 0.35;
    private static final double MAX_TURN_POWER = 0.30;
    // Floors so the robot doesn't stall just short of the target.
    private static final double MIN_DRIVE_POWER = 0.08;
    private static final double MIN_TURN_POWER = 0.08;
    private static final double TURN_TOLERANCE_DEG = 2.0;
    private static final double DRIVE_TOLERANCE_INCHES = 1.5;
    /** Closer than this, stop correcting heading (bearing gets noisy at tiny distances). */
    private static final double HEADING_FREEZE_INCHES = 3.0;
    private static final long TURN_TIMEOUT_MS = 4000;
    private static final long DRIVE_TIMEOUT_MS = 7000;

    private Follower follower;
    private OpenCvWebcam webcam;
    private EasyOBJDPipeline pipeline;
    private volatile boolean cameraReady = false;

    private String step = "";
    private String debug = "";
    private long lastTelemetryMs = 0;

    @SuppressLint("DiscouragedApi")
    @Override
    public void runOpMode() {
        AllianceConfig.reset();

        follower = Constants.createFollower(hardwareMap);
        Pose startPose = new Pose(0, 0, 0);
        follower.setStartingPose(startPose);

        initCamera();

        // ---------------- INIT: alliance selection ----------------
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b) {
                AllianceConfig.select(Alliance.RED);
            } else if (gamepad1.x || gamepad1.cross) {
                AllianceConfig.select(Alliance.BLUE);
            }
            follower.update();

            telemetry.addData("Status", "Select alliance, then START");
            telemetry.addLine("X = BLUE, O (Circle / B) = RED");
            telemetry.addData("Alliance", AllianceConfig.isSelected()
                    ? AllianceConfig.get().name()
                    : "NOT SELECTED (defaults to RED)");
            telemetry.addData("Camera", cameraReady ? "ready" : "starting...");
            telemetry.addData("Pose", poseString());
            telemetry.update();
        }
        if (isStopRequested()) {
            closeCamera();
            return;
        }

        if (!AllianceConfig.isSelected()) {
            AllianceConfig.select(Alliance.RED);
        }
        AllianceConfig.lock();
        Alliance alliance = AllianceConfig.get();

        // Must be called before any setTeleOpDrive() (otherwise it throws a NullPointerException).
        follower.startTeleopDrive();

        try {
            // ---------------- 1. Wait for the hive to tip ----------------
            step = "Alliance " + alliance.name() + " - waiting for hive to tip";
            waitMs(HIVE_WAIT_MS);

            // ---------------- 2. Turn: RED = left, BLUE = right ----------------
            double turnRad = Math.toRadians(TURN_DEGREES);
            double searchHeading = normalize(startPose.getHeading()
                    + (alliance == Alliance.RED ? turnRad : -turnRad));
            step = "Turning " + (alliance == Alliance.RED ? "left" : "right");
            turnToHeading(searchHeading);

            // ---------------- 3. Search here; if nothing, turn 180 and search again ----------------
            step = "Searching (1st direction)";
            ClusterInfo cluster = searchForCluster();
            Pose seenFrom = follower.getPose();

            if (cluster == null && opModeIsActive()) {
                step = "Nothing seen - turning 180";
                turnToHeading(normalize(searchHeading + Math.PI));

                step = "Searching (2nd direction)";
                cluster = searchForCluster();
                seenFrom = follower.getPose();
            }

            // ---------------- 4. Drive to the cluster ----------------
            if (cluster != null && opModeIsActive()) {
                driveToCluster(cluster, seenFrom);
            } else {
                step = "No cluster in either direction";
                debug = "";
            }

            // ---------------- 5. Return to the starting pose ----------------
            if (opModeIsActive()) {
                step = "Returning to start";
                driveToPoint(startPose.getX(), startPose.getY());
                step = "Facing start heading";
                turnToHeading(startPose.getHeading());
            }

            step = "Done";
            stopDrive();
            telemetryTick(true);
        } finally {
            stopDrive();
            closeCamera();
        }
    }

    // =====================================================================
    // Cluster -> field point -> drive
    // =====================================================================

    private void driveToCluster(ClusterInfo cluster, Pose robot) {
        if (Double.isNaN(cluster.x) || Double.isNaN(cluster.y)) {
            step = "Cluster position is NaN - check camera rotation/tilt/height/focal";
            debug = "x=" + cluster.x + " y=" + cluster.y;
            sleepWithTelemetry(2500);
            return;
        }

        double h = robot.getHeading();
        double fx = Math.cos(h);   // robot forward, in field coordinates
        double fy = Math.sin(h);
        double lx = -Math.sin(h);  // robot left, in field coordinates
        double ly = Math.cos(h);

        double aheadOfCenter = LENS_FORWARD_OF_CENTER_INCHES + cluster.y; // forward from robot center
        double left = -cluster.x;                                         // camera x is RIGHT, so flip

        double clusterX = robot.getX() + aheadOfCenter * fx + left * lx;
        double clusterY = robot.getY() + aheadOfCenter * fy + left * ly;

        double dx = clusterX - robot.getX();
        double dy = clusterY - robot.getY();
        double dist = Math.hypot(dx, dy);
        double travel = dist - STOP_SHORT_INCHES;
        double bearing = Math.atan2(dy, dx);

        debug = String.format(
                "cam x %.1f y %.1f | balls %d | field %.1f, %.1f | dist %.1f | travel %.1f",
                cluster.x, cluster.y, cluster.ballCount, clusterX, clusterY, dist, travel);

        if (travel < 1.0) {
            step = "Cluster already within stop distance - not moving";
            sleepWithTelemetry(2000);
            return;
        }

        // Stop point along the line to the cluster.
        double targetX = robot.getX() + travel * Math.cos(bearing);
        double targetY = robot.getY() + travel * Math.sin(bearing);

        step = "Facing cluster";
        turnToHeading(bearing);
        step = "Driving to cluster";
        driveToPoint(targetX, targetY);
    }

    // =====================================================================
    // Motion (proportional control on the Pedro pose, via teleop drive)
    // =====================================================================

    /** Rotates in place to an absolute field heading (radians). */
    private void turnToHeading(double targetRad) {
        long end = System.currentTimeMillis() + TURN_TIMEOUT_MS;
        int settled = 0;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            follower.update();
            double errDeg = Math.toDegrees(normalize(targetRad - follower.getPose().getHeading()));

            double turn = 0;
            if (Math.abs(errDeg) <= TURN_TOLERANCE_DEG) {
                settled++;
            } else {
                settled = 0;
                turn = signedPower(TURN_KP * errDeg, MIN_TURN_POWER, MAX_TURN_POWER);
            }
            if (settled >= 3) {
                break;
            }
            // Positive turn = counter-clockwise (left), positive error = target is to the left.
            follower.setTeleOpDrive(0, 0, turn, true);
            debug = String.format("heading err %.1f deg, turn power %.2f", errDeg, turn);
            telemetryTick(false);
        }
        stopDrive();
    }

    /** Drives to a field point: forward power from distance, turn power from bearing error. */
    private void driveToPoint(double targetX, double targetY) {
        long end = System.currentTimeMillis() + DRIVE_TIMEOUT_MS;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            follower.update();
            Pose p = follower.getPose();
            double dx = targetX - p.getX();
            double dy = targetY - p.getY();
            double dist = Math.hypot(dx, dy);
            if (dist <= DRIVE_TOLERANCE_INCHES) {
                break;
            }

            double errRad = normalize(Math.atan2(dy, dx) - p.getHeading());
            double errDeg = Math.toDegrees(errRad);

            double turn = 0;
            if (dist > HEADING_FREEZE_INCHES && Math.abs(errDeg) > TURN_TOLERANCE_DEG) {
                turn = signedPower(TURN_KP * errDeg, MIN_TURN_POWER, MAX_TURN_POWER);
            }

            // Forward only when roughly facing the target, so the robot turns first if it is way off.
            double forward = signedPower(FORWARD_KP * dist, MIN_DRIVE_POWER, MAX_DRIVE_POWER)
                    * Math.max(0, Math.cos(errRad));

            follower.setTeleOpDrive(forward, 0, turn, true);
            debug = String.format("dist %.1f in | bearing err %.1f deg | fwd %.2f turn %.2f",
                    dist, errDeg, forward, turn);
            telemetryTick(false);
        }
        stopDrive();
    }

    private void stopDrive() {
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    /** Keeps the follower updating (so the pose stays live) while waiting. */
    private void waitMs(long ms) {
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            follower.update();
            telemetryTick(false);
        }
    }

    private void sleepWithTelemetry(long ms) {
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            follower.update();
            telemetryTick(false);
        }
    }

    private static double signedPower(double value, double min, double max) {
        double mag = Math.max(min, Math.min(max, Math.abs(value)));
        return Math.signum(value) * mag;
    }

    private static double normalize(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians <= -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    private String poseString() {
        Pose p = follower.getPose();
        return String.format("x %.1f  y %.1f  heading %.1f deg",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
    }

    /** Sends telemetry at most every 100 ms so it never starves the control loop. */
    private void telemetryTick(boolean force) {
        long now = System.currentTimeMillis();
        if (!force && now - lastTelemetryMs < 100) {
            return;
        }
        lastTelemetryMs = now;
        telemetry.addData("Step", step);
        telemetry.addData("Pose", poseString());
        telemetry.addData("Detail", debug);
        telemetry.update();
    }

    // =====================================================================
    // Vision
    // =====================================================================

    @SuppressLint("DiscouragedApi")
    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);

        pipeline = EasyOBJD.createPipeline();
        EasyOBJDConfig cfg = pipeline.getConfig();
        cfg.cameraHeightInches = CAMERA_HEIGHT_INCHES;
        cfg.ballDiameterInches = BALL_DIAMETER_INCHES;
        cfg.horizontalFovDegrees = HORIZONTAL_FOV_DEGREES;
        cfg.cameraTiltDegrees = CAMERA_TILT_DEGREES;
        cfg.focalLengthPixelsAt640 = FOCAL_LENGTH_PIXELS_AT_640;
        cfg.overlayMode = OverlayMode.FULL;
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, CAMERA_ROTATION,
                        OpenCvWebcam.StreamFormat.MJPEG);
                cameraReady = true;
            }

            @Override
            public void onError(int errorCode) {
                cameraReady = false;
                telemetry.addData("Camera Error", errorCode);
            }
        });
    }

    private void closeCamera() {
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    /**
     * Looks for up to SEARCH_MS. As soon as a cluster shows up it reads a few more
     * frames, then returns the largest one (most balls, ties go to the closer).
     * Returns null if nothing was seen in time.
     */
    private ClusterInfo searchForCluster() {
        long startTime = System.currentTimeMillis();
        long settleUntilFrame = -1;

        while (opModeIsActive() && System.currentTimeMillis() - startTime < SEARCH_MS) {
            follower.update();
            telemetryTick(false);

            if (settleUntilFrame < 0 && !pipeline.getClusters().isEmpty()) {
                settleUntilFrame = pipeline.getFrameCount() + SETTLE_FRAMES;
            }
            if (settleUntilFrame >= 0 && pipeline.getFrameCount() >= settleUntilFrame) {
                break;
            }
        }
        return largest(pipeline.getClusters());
    }

    private static ClusterInfo largest(List<ClusterInfo> clusters) {
        ClusterInfo best = null;
        for (ClusterInfo c : clusters) {
            if (best == null
                    || c.ballCount > best.ballCount
                    || (c.ballCount == best.ballCount && c.rangeInches < best.rangeInches)) {
                best = c;
            }
        }
        return best;
    }
}