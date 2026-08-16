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

import java.util.List;

/**
 * Autonomous for Off Season Bot 1 (OFSB1).
 *
 * Scans for the closest yellow Pollen ball via OFSB1VisionProcessor, tracks
 * it across frames to confirm it's stable, then:
 *   1) TURNS in place to face the ball's direction
 *   2) DRIVES straight forward to a point TARGET_DISTANCE_INCHES short of it
 * Splitting into turn-then-drive means the drive phase is a pure forward
 * move along the robot's heading - no strafe component needed, since the
 * robot is already pointed at the ball before it starts translating.
 */
@Autonomous(name = "OFSB1 Auto", group = "OFSB1")
public class OFSB1Auto extends OpMode {

    private static final double TARGET_DISTANCE_INCHES = 5.0;
    // How many consecutive frames the SAME ball must be seen before we trust
    // it and commit to a path - filters out one-frame noise blobs.
    private static final int CONFIRM_FRAMES = 5;
    // How far (inches, camera-relative x/z) a detection can be from the
    // previous frame's candidate and still count as "the same ball" - keeps
    // us from bouncing between two nearby balls frame to frame.
    private static final double MATCH_DISTANCE_INCHES = 4.0;
    // How many consecutive frames the candidate ball is allowed to vanish
    // (occlusion, one bad mask frame) before we give up and reset - keeps
    // a single dropped frame from throwing away confirm progress.
    private static final int MAX_MISSED_FRAMES = 3;

    private Follower follower;
    private OFSB1Subsystem robot;
    private OpenCvWebcam webcam;
    private OFSB1VisionProcessor visionProcessor;
    private volatile boolean cameraInitialized = false;

    private enum State { SCANNING, DRIVING, DONE }
    private State state = State.SCANNING;
    private int confirmCount = 0;
    private int missedFrames = 0;
    // The ball we are currently tracking toward confirmation - null until
    // we've seen at least one candidate.
    private OFSB1VisionProcessor.Detection candidate = null;
    // The ball we actually locked onto. Kept for telemetry after locking,
    // since we stop reading fresh detections at that point.
    private OFSB1VisionProcessor.Detection lockedTarget = null;

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
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
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
        int ballCount = visionProcessor.getDetections().size();

        telemetry.addData("Camera Initialized", cameraInitialized);
        telemetry.addData("Ball Count", ballCount);

        if (ballCount == 0) {
            telemetry.addLine("No balls detected - check lighting/camera aim");
        } else {
            List<OFSB1VisionProcessor.Detection> balls = visionProcessor.getDetections();
            for (int i = 0; i < balls.size(); i++) {
                OFSB1VisionProcessor.Detection b = balls.get(i);
                telemetry.addData("Ball " + i, String.format("x=%.1f z=%.1f", b.x, b.z));
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
                scanForBall();
                break;
            case DRIVING:
                if (!follower.isBusy()) {
                    state = State.DONE;
                }
                break;
            case DONE:
                telemetry.addLine("Path Complete - stopped near ball");
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Camera Initialized", cameraInitialized);

        if (state == State.SCANNING) {
            // Still hunting - this number legitimately fluctuates frame to
            // frame, that's expected and fine, it's not driving anything.
            telemetry.addData("Ball Count (live)", visionProcessor.getDetections().size());
        } else if (lockedTarget != null) {
            // Locked - show the frozen target we committed to, not live
            // detections, since we've stopped reading the camera for this.
            telemetry.addData("Locked Ball X (in)", "%.1f", lockedTarget.x);
            telemetry.addData("Locked Ball Z (in)", "%.1f", lockedTarget.z);
        }

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    private void scanForBall() {
        List<OFSB1VisionProcessor.Detection> balls = visionProcessor.getDetections();

        OFSB1VisionProcessor.Detection match = findMatch(balls);

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

        // Either confirms the existing candidate (same ball, close enough
        // to its last known position) or starts tracking a brand new one.
        boolean sameCandidate = candidate != null
                && Math.hypot(match.x - candidate.x, match.z - candidate.z) <= MATCH_DISTANCE_INCHES;

        if (!sameCandidate) {
            candidate = match;
            confirmCount = 1;
        } else {
            candidate = match; // update to latest position of the same ball
            confirmCount++;
        }
        missedFrames = 0;

        telemetry.addData("Status", "Tracking ball, confirming (" + confirmCount + "/" + CONFIRM_FRAMES + ")");
        telemetry.addData("Ball Count (live)", balls.size());
        telemetry.addData("Candidate X (in)", "%.1f", candidate.x);
        telemetry.addData("Candidate Z (in)", "%.1f", candidate.z);

        if (confirmCount < CONFIRM_FRAMES) {
            return; // keep tracking until confirmed stable
        }

        // Locked in - freeze this detection and stop scanning. buildAndFollowPathToBall
        // moves state out of SCANNING, so scanForBall() will not run again this OpMode run.
        lockedTarget = candidate;
        buildAndFollowPathToBall(lockedTarget);
    }

    /**
     * Finds the detection this frame that best matches what we're tracking.
     * If we already have a candidate, prefer whichever ball is closest to
     * its last known position (so we don't jump to a different ball just
     * because it's momentarily nearer the camera). If we have no candidate
     * yet, fall back to the globally closest ball, same as before.
     */
    private OFSB1VisionProcessor.Detection findMatch(List<OFSB1VisionProcessor.Detection> balls) {
        if (balls.isEmpty()) return null;

        if (candidate == null) {
            OFSB1VisionProcessor.Detection closest = balls.get(0);
            for (OFSB1VisionProcessor.Detection b : balls) {
                if (b.z < closest.z) closest = b;
            }
            return closest;
        }

        OFSB1VisionProcessor.Detection best = null;
        double bestDist = Double.MAX_VALUE;
        for (OFSB1VisionProcessor.Detection b : balls) {
            double dist = Math.hypot(b.x - candidate.x, b.z - candidate.z);
            if (dist < bestDist) {
                bestDist = dist;
                best = b;
            }
        }
        // Only accept as a match if it's actually near where we expect the
        // tracked ball to be - otherwise treat it as "not seen this frame"
        // rather than silently snapping to an unrelated ball.
        return (bestDist <= MATCH_DISTANCE_INCHES) ? best : null;
    }

    /**
     * Builds and follows a single path to the ball, using TANGENTIAL heading
     * interpolation: the follower automatically rotates to face the
     * direction it's currently traveling along the path, rather than being
     * told a fixed target angle. For a straight BezierLine to the ball, this
     * means the robot turns to face the ball as it starts moving, driving
     * straight toward it with no separate turn-in-place step and no strafe
     * component needed once it's underway.
     */
    private void buildAndFollowPathToBall(OFSB1VisionProcessor.Detection target) {
        Pose robotPose = follower.getPose();

        // Convert camera-relative offset into a field-relative bearing/distance.
        // x: lateral offset (in), z: forward distance (in), both camera-relative.
        double angleOffsetRadians = Math.atan2(target.x, target.z);
        double distanceToBall = Math.hypot(target.x, target.z);
        double driveDistance = distanceToBall - TARGET_DISTANCE_INCHES;

        double fieldAngle = robotPose.getHeading() + angleOffsetRadians;

        // Both target coordinates are computed in the SAME field frame as
        // robotPose - do not negate one axis without negating the other, or
        // start/end points end up in mismatched coordinate frames.
        double targetX = robotPose.getX() + driveDistance * Math.cos(fieldAngle);
        double targetY = robotPose.getY() + driveDistance * Math.sin(fieldAngle);
        // Heading value here is mostly irrelevant for tangential interpolation
        // (the path direction determines heading, not this field), but PedroPathing's
        // Pose constructor requires a heading argument - fieldAngle is a reasonable one.
        Pose targetPose = new Pose(targetX, targetY, fieldAngle);

        Path pathToBall = new Path(new BezierLine(robotPose, targetPose));
        // NOTE: verify this method name against your PedroPathing version -
        // some releases call this setTangentHeadingInterpolation(), others
        // may expose it as a HeadingInterpolator.tangent() passed to a
        // general setHeadingInterpolation() setter. Check your Path Javadoc
        // if this doesn't compile.
        pathToBall.setTangentHeadingInterpolation();

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