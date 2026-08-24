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
@Autonomous(name = "OFSB1 Auto", group = "OFSB1")
public class OFSB1Auto extends OpMode {

    private static final double TARGET_DISTANCE_INCHES = 12.0;
    //stops before hitting the ball
    private static final double PATH_TARGET_SAFETY_MARGIN_INCHES = 1.0;
    //how mqny times the frame gets detected before ball positon can be trusted
    private static final int CONFIRM_FRAMES = 3;
    // How far (inches, camera-relative x/z) a detection
    private static final double MATCH_DISTANCE_INCHES = 4.0;
    //how mqny missed frames
    private static final int MAX_MISSED_FRAMES = 3;

    // Only send telemetry.update() (network I/O) every N loop iterations,
    // instead of every single loop, so the control loop isn't throttled
    // waiting on the Driver Station connection. addData() calls are cheap
    // and still happen every loop - only the actual send is skipped.
    private static final int TELEMETRY_UPDATE_INTERVAL = 5;
    private int telemetryCounter = 0;

    private Follower follower;
    private OFSB1Subsystem robot;
    private OpenCvWebcam webcam;
    private OFSB1VisionProcessor visionProcessor;
    private volatile boolean cameraInitialized = false;

    private enum State { SCANNING, DRIVING, DONE }
    private State state = State.SCANNING;
    private int confirmCount = 0;
    private int missedFrames = 0;
    //ball being tracked till confirmation, untill then its null
    private OFSB1VisionProcessor.Detection candidate = null;
    // stop reading more frames once telemetry is locked onto ball position
    private OFSB1VisionProcessor.Detection lockedTarget = null;
    // Tracks the most recent distance seen to ANY ball while DRIVING. If the
    // ball vanishes from vision (too close, out of frame, blurred) consideered "safe"
    private double lastSeenCloseZ = Double.MAX_VALUE;
    // If the ball was last seen closer than this when it disappeared,
    // assume it's about to be hit and stop immediately.
    private static final double LOST_TRACKING_STOP_THRESHOLD_INCHES = 15.0;

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
        lastSeenCloseZ = Double.MAX_VALUE;
        telemetryCounter = 0;
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case SCANNING:
                scanForBall();
                break;
            case DRIVING:
                if (checkLiveSafetyStop()) {
                    state = State.DONE;
                } else if (!follower.isBusy()) {
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

        // Throttle the actual network send - addData() above is cheap and
        // still runs every loop, but telemetry.update() is comparatively
        // slow and doesn't need to fire every single cycle.
        telemetryCounter++;
        if (telemetryCounter >= TELEMETRY_UPDATE_INTERVAL) {
            telemetry.update();
            telemetryCounter = 0;
        }
    }

    /**
     * Live safety check during DRIVING. Two trigger conditions, either of
     * which force-stops the robot immediately:
     *   1) A ball is currently seen at or inside TARGET_DISTANCE_INCHES.
     *   2) The ball WAS recently seen close (within LOST_TRACKING_STOP_
     *      THRESHOLD_INCHES) and has now vanished from detection entirely -
     *      at close range this almost always means the ball went out of
     *      frame or out of focus right before contact, NOT that it's
     *      actually safe. Treating "no detection" as "safe" here is exactly
     *      backwards and is the likely reason earlier versions still hit
     *      the ball.
     *
     * On trigger, forces zero drive power directly rather than trusting
     * breakFollowing() alone to zero the motors - a belt-and-suspenders
     * guarantee against any lag between "path cancelled" and "wheels
     * actually stopped."
     */
    private boolean checkLiveSafetyStop() {
        List<OFSB1VisionProcessor.Detection> balls = visionProcessor.getDetections();

        if (!balls.isEmpty()) {
            double closestZ = Double.MAX_VALUE;
            for (OFSB1VisionProcessor.Detection b : balls) {
                if (b.z < closestZ) closestZ = b.z;
            }
            lastSeenCloseZ = closestZ;

            if (closestZ <= TARGET_DISTANCE_INCHES) {
                forceStop("SAFETY STOP - ball within target distance (z=" + String.format("%.1f", closestZ) + ")");
                return true;
            }
            return false;
        }

        // No balls detected this frame - if it was closing in fast before
        // vanishing, assume it's now too close to see rather than gone.
        if (lastSeenCloseZ <= LOST_TRACKING_STOP_THRESHOLD_INCHES) {
            forceStop("SAFETY STOP - ball lost from view at close range (last z=" + String.format("%.1f", lastSeenCloseZ) + ")");
            return true;
        }
        return false;
    }

    private void forceStop(String reason) {
        // NOTE: verify breakFollowing() against your PedroPathing version -
        // it should immediately cancel/abort the currently-following path.
        follower.breakFollowing();
        // Explicit zero-power override as a hard guarantee, independent of
        // whether breakFollowing() alone zeroes the drivetrain instantly.
        follower.setTeleOpDrive(0, 0, 0, true);
        telemetry.addLine(reason);
        // Safety-critical message - force an immediate send rather than
        // waiting on the throttled telemetry cadence.
        telemetry.update();
        telemetryCounter = 0;
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
     * Builds and follows a single path to the ball, using CONSTANT heading
     * interpolation: the robot holds a fixed heading (facing the ball's
     * bearing) for the entire path, using strafe as needed to translate
     * diagonally if the ball isn't directly ahead. driveDistance is
     * distanceToBall - TARGET_DISTANCE_INCHES, so the path's endpoint sits
     * exactly TARGET_DISTANCE_INCHES short of the ball along the
     * line-of-sight - the robot stops there instead of driving into it.
     */
    private void buildAndFollowPathToBall(OFSB1VisionProcessor.Detection target) {
        Pose robotPose = follower.getPose();

        // Convert camera-relative offset into a field-relative bearing/distance.
        // x: lateral offset (in), z: forward distance (in), both camera-relative.
        double angleOffsetRadians = Math.atan2(target.x, target.z);
        double distanceToBall = Math.hypot(target.x, target.z);
        double driveDistance = distanceToBall - TARGET_DISTANCE_INCHES - PATH_TARGET_SAFETY_MARGIN_INCHES;

        double fieldAngle = robotPose.getHeading() + angleOffsetRadians;

        // Both target coordinates are computed in the SAME field frame as
        // robotPose - do not negate one axis without negating the other, or
        // start/end points end up in mismatched coordinate frames.
        double targetX = robotPose.getX() + driveDistance * Math.cos(fieldAngle);
        double targetY = robotPose.getY() + driveDistance * Math.sin(fieldAngle);
        Pose targetPose = new Pose(targetX, targetY, fieldAngle);

        Path pathToBall = new Path(new BezierLine(robotPose, targetPose));
        pathToBall.setConstantHeadingInterpolation(fieldAngle);

        lastSeenCloseZ = Double.MAX_VALUE;
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