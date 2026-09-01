package org.firstinspires.ftc.teamcode.OFSB1.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OFSB1.Constants;
import org.firstinspires.ftc.teamcode.OFSB1.Subsystems.OFSB1Subsystem;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.OFSB1VisionProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

/**
 * Autonomous for Off Season Bot 1 (OFSB1).
 *
 * Two selectable behaviors - pick with gamepad1 during init (shown on
 * telemetry), locked in at start:
 *
 *   MODE 1 (press A): SINGLE BALL - track the closest yellow Pollen ball,
 *     drive to TARGET_DISTANCE_INCHES short of it, facing it.
 *
 *   MODE 2 (press B): BIGGEST CLUSTER - group detected balls into clusters
 *     (balls within CLUSTER_LINK_INCHES of each other belong to the same
 *     blob), then drive to the centroid of the cluster containing the MOST
 *     balls, stopping TARGET_DISTANCE_INCHES short and facing it.
 *
 * Both modes use the same confirm-over-several-frames tracking so a
 * one-frame noise detection never triggers a path.
 */
@Autonomous(name = "OFSB1 Auto", group = "OFSB1")
public class OFSB1Auto extends OpMode {

    // How far the FRONT OF THE ROBOT should stop from the target (inches).
    private static final double TARGET_DISTANCE_INCHES = 5.0;

    // ---- Camera mounting geometry (inches) ----
    // Measured robot: 14.5 long (camera 8 from the back, ~6.5 behind the
    // front), 13.5 wide (camera 7 from the left, 6.5 from the right).
    // PedroPathing poses track the ROBOT CENTER, but the vision processor
    // reports offsets from the CAMERA LENS - these convert between the two.
    private static final double CAMERA_FORWARD_OF_CENTER = 0.75; // 8 - 14.5/2
    private static final double CAMERA_RIGHT_OF_CENTER = 0.25;   // 7 - 13.5/2
    private static final double CENTER_TO_FRONT = 7.25;          // 14.5/2
    // How many consecutive frames the SAME target must be seen before we
    // trust it and commit to a path - filters out one-frame noise blobs.
    private static final int CONFIRM_FRAMES = 5;
    // How far (inches, camera-relative x/z) a detection can be from the
    // previous frame's candidate and still count as "the same target".
    private static final double MATCH_DISTANCE_INCHES = 4.0;
    // Cluster centroids wobble more than a single ball (membership can
    // change frame to frame), so allow a looser match for them.
    private static final double CLUSTER_MATCH_DISTANCE_INCHES = 6.0;
    // How many consecutive frames the candidate is allowed to vanish
    // (occlusion, one bad mask frame) before we give up and reset.
    private static final int MAX_MISSED_FRAMES = 3;
    // Two balls whose camera-relative positions are within this distance of
    // each other count as part of the same cluster/blob (Mode 2).
    private static final double CLUSTER_LINK_INCHES = 10.0;

    private enum Mode { SINGLE_BALL, BIGGEST_CLUSTER }
    private Mode mode = Mode.SINGLE_BALL;

    private Follower follower;
    private OFSB1Subsystem robot;
    private OpenCvWebcam webcam;
    private OFSB1VisionProcessor visionProcessor;
    private volatile boolean cameraInitialized = false;

    private enum State { SCANNING, DRIVING, DONE }
    private State state = State.SCANNING;
    private int confirmCount = 0;
    private int missedFrames = 0;

    /**
     * What we drive toward. In SINGLE_BALL mode this is one detection; in
     * BIGGEST_CLUSTER mode it's the centroid of a group of detections.
     */
    private static class Target {
        double x;      // camera-relative lateral offset, inches (+ right)
        double z;      // camera-relative forward distance, inches
        int ballCount; // 1 in single-ball mode; cluster size in cluster mode
    }

    // The target we are currently tracking toward confirmation.
    private Target candidate = null;
    // The target we locked onto and committed a path to (kept for telemetry).
    private Target lockedTarget = null;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        robot = new OFSB1Subsystem(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        visionProcessor = new OFSB1VisionProcessor();
        webcam.setPipeline(visionProcessor);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                cameraInitialized = true;
            }

            @Override
            public void onError(int errorCode) {
                cameraInitialized = false;
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Status", "OFSB1 Auto Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // ---- Mode selection (locked in once START is pressed) ----
        if (gamepad1.a) mode = Mode.SINGLE_BALL;
        if (gamepad1.b) mode = Mode.BIGGEST_CLUSTER;

        telemetry.addLine("=== MODE SELECT (gamepad1) ===");
        telemetry.addLine("A = Mode 1: closest single ball");
        telemetry.addLine("B = Mode 2: cluster with most balls");
        telemetry.addData("Selected", mode == Mode.SINGLE_BALL
                ? "Mode 1: SINGLE BALL" : "Mode 2: BIGGEST CLUSTER");
        telemetry.addLine("");

        // ---- Live vision preview ----
        List<OFSB1VisionProcessor.Detection> balls = visionProcessor.getDetections();
        telemetry.addData("Camera Initialized", cameraInitialized);
        telemetry.addData("Ball Count", balls.size());

        if (balls.isEmpty()) {
            telemetry.addLine("No balls detected - check lighting/camera aim");
        } else {
            for (int i = 0; i < balls.size(); i++) {
                OFSB1VisionProcessor.Detection b = balls.get(i);
                telemetry.addData("Ball " + i, String.format("x=%.1f z=%.1f", b.x, b.z));
            }
            List<Target> clusters = clusterBalls(balls);
            telemetry.addData("Cluster Count", clusters.size());
            Target biggest = null;
            for (Target t : clusters) {
                if (biggest == null || t.ballCount > biggest.ballCount) biggest = t;
            }
            if (biggest != null) {
                telemetry.addData("Biggest Cluster",
                        String.format("%d balls at x=%.1f z=%.1f", biggest.ballCount, biggest.x, biggest.z));
            }
        }

        telemetry.update();
    }

    @Override
    public void start() {
        state = State.SCANNING;
        confirmCount = 0;
        missedFrames = 0;
        candidate = null;
        lockedTarget = null;
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case SCANNING:
                scanForTarget();
                break;
            case DRIVING:
                if (!follower.isBusy()) {
                    state = State.DONE;
                }
                break;
            case DONE:
                telemetry.addLine("Path Complete - stopped near target");
                break;
        }

        telemetry.addData("Mode", mode);
        telemetry.addData("State", state);
        telemetry.addData("Camera Initialized", cameraInitialized);

        if (state == State.SCANNING) {
            // Still hunting - this number legitimately fluctuates frame to
            // frame, that's expected and fine, it's not driving anything.
            telemetry.addData("Ball Count (live)", visionProcessor.getDetections().size());
        } else if (lockedTarget != null) {
            // Locked - show the frozen target we committed to, not live
            // detections, since we've stopped reading the camera for this.
            telemetry.addData("Locked Target X (in)", "%.1f", lockedTarget.x);
            telemetry.addData("Locked Target Z (in)", "%.1f", lockedTarget.z);
            telemetry.addData("Locked Target Balls", lockedTarget.ballCount);
        }

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    private void scanForTarget() {
        List<OFSB1VisionProcessor.Detection> balls = visionProcessor.getDetections();

        // Build this frame's list of possible targets for the active mode.
        List<Target> targets = (mode == Mode.SINGLE_BALL)
                ? ballsAsTargets(balls)
                : clusterBalls(balls);
        double matchDist = (mode == Mode.SINGLE_BALL)
                ? MATCH_DISTANCE_INCHES
                : CLUSTER_MATCH_DISTANCE_INCHES;

        Target match = findMatch(targets, matchDist);

        // In cluster mode, if a different cluster now clearly holds more
        // balls than the one we're tracking, abandon ours and start
        // confirming the bigger one - "most balls" is the whole point.
        if (mode == Mode.BIGGEST_CLUSTER && match != null) {
            Target biggest = pickBest(targets);
            if (biggest != null && biggest.ballCount > match.ballCount) {
                candidate = null; // force re-confirmation from scratch
                match = biggest;
            }
        }

        if (match == null) {
            // Candidate not seen this frame - tolerate a few missed frames
            // (occlusion, one bad mask) before giving up on it entirely.
            missedFrames++;
            telemetry.addData("Status", "No match (" + missedFrames + "/" + MAX_MISSED_FRAMES + " missed)");
            telemetry.addData("Ball Count (live)", balls.size());
            if (missedFrames > MAX_MISSED_FRAMES) {
                confirmCount = 0;
                candidate = null;
            }
            return;
        }

        // Either confirms the existing candidate (same target, close enough
        // to its last known position) or starts tracking a brand new one.
        boolean sameCandidate = candidate != null
                && Math.hypot(match.x - candidate.x, match.z - candidate.z) <= matchDist;

        if (!sameCandidate) {
            candidate = match;
            confirmCount = 1;
        } else {
            candidate = match; // update to latest position of the same target
            confirmCount++;
        }
        missedFrames = 0;

        telemetry.addData("Status", "Tracking target, confirming (" + confirmCount + "/" + CONFIRM_FRAMES + ")");
        telemetry.addData("Ball Count (live)", balls.size());
        telemetry.addData("Candidate X (in)", "%.1f", candidate.x);
        telemetry.addData("Candidate Z (in)", "%.1f", candidate.z);
        telemetry.addData("Candidate Balls", candidate.ballCount);

        if (confirmCount < CONFIRM_FRAMES) {
            return; // keep tracking until confirmed stable
        }

        // Locked in - freeze this target and stop scanning. buildAndFollowPathToTarget
        // moves state out of SCANNING, so scanForTarget() will not run again this run.
        lockedTarget = candidate;
        buildAndFollowPathToTarget(lockedTarget);
    }

    /** Wraps each individual detection as its own Target (Mode 1). */
    private List<Target> ballsAsTargets(List<OFSB1VisionProcessor.Detection> balls) {
        List<Target> out = new ArrayList<>();
        for (OFSB1VisionProcessor.Detection b : balls) {
            Target t = new Target();
            t.x = b.x;
            t.z = b.z;
            t.ballCount = 1;
            out.add(t);
        }
        return out;
    }

    /**
     * Groups detections into clusters: a ball joins a cluster if it is
     * within CLUSTER_LINK_INCHES of ANY ball already in it (Mode 2). Each
     * cluster becomes one Target at the centroid of its members.
     *
     * Greedy single-pass grouping - good enough for the handful of balls
     * ever visible at once; not worth a full union-find.
     */
    private List<Target> clusterBalls(List<OFSB1VisionProcessor.Detection> balls) {
        List<List<OFSB1VisionProcessor.Detection>> groups = new ArrayList<>();

        for (OFSB1VisionProcessor.Detection b : balls) {
            List<OFSB1VisionProcessor.Detection> home = null;
            for (List<OFSB1VisionProcessor.Detection> g : groups) {
                for (OFSB1VisionProcessor.Detection m : g) {
                    if (Math.hypot(b.x - m.x, b.z - m.z) <= CLUSTER_LINK_INCHES) {
                        home = g;
                        break;
                    }
                }
                if (home != null) break;
            }
            if (home == null) {
                home = new ArrayList<>();
                groups.add(home);
            }
            home.add(b);
        }

        List<Target> out = new ArrayList<>();
        for (List<OFSB1VisionProcessor.Detection> g : groups) {
            Target t = new Target();
            for (OFSB1VisionProcessor.Detection m : g) {
                t.x += m.x;
                t.z += m.z;
            }
            t.x /= g.size();
            t.z /= g.size();
            t.ballCount = g.size();
            out.add(t);
        }
        return out;
    }

    /**
     * Finds this frame's target that best matches what we're tracking.
     * If we already have a candidate, prefer whichever target is closest to
     * its last known position - but only within matchDist, otherwise report
     * "not seen" rather than silently snapping to something unrelated.
     * With no candidate yet, pick the best target for the active mode.
     */
    private Target findMatch(List<Target> targets, double matchDist) {
        if (targets.isEmpty()) return null;

        if (candidate == null) {
            return pickBest(targets);
        }

        Target best = null;
        double bestDist = Double.MAX_VALUE;
        for (Target t : targets) {
            double dist = Math.hypot(t.x - candidate.x, t.z - candidate.z);
            if (dist < bestDist) {
                bestDist = dist;
                best = t;
            }
        }
        return (bestDist <= matchDist) ? best : null;
    }

    /**
     * Best fresh target for the active mode: closest ball in SINGLE_BALL,
     * most balls (ties broken by distance) in BIGGEST_CLUSTER.
     */
    private Target pickBest(List<Target> targets) {
        Target best = null;
        for (Target t : targets) {
            if (best == null) {
                best = t;
            } else if (mode == Mode.BIGGEST_CLUSTER) {
                if (t.ballCount > best.ballCount
                        || (t.ballCount == best.ballCount && t.z < best.z)) {
                    best = t;
                }
            } else if (t.z < best.z) {
                best = t;
            }
        }
        return best;
    }

    private void buildAndFollowPathToTarget(Target target) {
        Pose robotPose = follower.getPose();

        // Shift the camera-relative reading to be ROBOT-CENTER-relative,
        // since that's what the follower's pose refers to.
        double ballForward = target.z + CAMERA_FORWARD_OF_CENTER;
        double ballRight = target.x + CAMERA_RIGHT_OF_CENTER;

        double distanceToBall = Math.hypot(ballRight, ballForward);
        // Stop so the FRONT BUMPER (7.25in ahead of center once we're
        // facing the ball) ends up TARGET_DISTANCE_INCHES from the target -
        // NOT the camera, which sits ~6.5in behind the front and would
        // otherwise let the robot plow into the ball.
        double driveDistance = distanceToBall - (CENTER_TO_FRONT + TARGET_DISTANCE_INCHES);

        if (driveDistance <= 0) {
            // Already at or inside the stop distance - nothing to drive.
            state = State.DONE;
            return;
        }

        // Ball position in FIELD coordinates via a proper rotation by the
        // robot's heading (CCW-positive; at heading 0, forward = +x and
        // left = +y). This replaces the old "+atan2 offset with a Y sign
        // flip" shortcut, which field testing (AprilTagAlignment's square
        // button) proved drives the wrong way once heading isn't zero.
        double h = robotPose.getHeading();
        double ballFieldX = robotPose.getX() + ballForward * Math.cos(h) + ballRight * Math.sin(h);
        double ballFieldY = robotPose.getY() + ballForward * Math.sin(h) - ballRight * Math.cos(h);

        double fieldAngle = Math.atan2(ballFieldY - robotPose.getY(), ballFieldX - robotPose.getX());

        double targetX = robotPose.getX() + driveDistance * Math.cos(fieldAngle);
        double targetY = robotPose.getY() + driveDistance * Math.sin(fieldAngle);
        Pose targetPose = new Pose(targetX, targetY, fieldAngle);

        // Constant heading along the line of sight = robot ends up FACING
        // the target, as required by both modes.
        Path pathToBall = new Path(new BezierLine(robotPose, targetPose));
        pathToBall.setConstantHeadingInterpolation(fieldAngle);

        follower.followPath(pathToBall);
        state = State.DRIVING;
    }

    @Override
    public void stop() {
        robot.stopAll();
        if (webcam != null) {
            webcam.stopStreaming();
        }
    }
}
