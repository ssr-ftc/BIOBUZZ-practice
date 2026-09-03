package org.firstinspires.ftc.teamcode.OFSB1.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
 * Continuously tracks and follows a single ball, holding a constant
 * standoff distance (DESIRED_DISTANCE_INCHES) as the ball moves around
 * the field.
 *
 * Unlike OFSB1Auto (which builds ONE path to a fixed point and stops),
 * this OpMode re-reads the ball's camera-relative position EVERY loop
 * and drives with proportional (visual-servo) power. Centering is done
 * by ROTATING to face the ball - not by strafing - so the ball is kept
 * on the camera's vertical centerline by turning, while forward/back
 * power independently closes or holds DESIRED_DISTANCE_INCHES along
 * whatever heading the robot currently faces. There is no destination
 * pose - "where to go" is recomputed fresh every cycle from whatever the
 * camera sees right now, so the robot keeps re-facing and re-ranging on
 * a moving target instead of committing to a single point like
 * OFSB1Auto does.
 */
@Autonomous(name = "OFSB1 Follow Ball", group = "OFSB1")
public class followerOFSB1auto extends OpMode {

    // How far from the ball the robot should try to stay, in inches.
    private static final double DESIRED_DISTANCE_INCHES = 5.0;

    // How many consecutive frames the ball must be seen in roughly the same
    // spot before we trust it enough to start driving. The vision pipeline
    // runs once per loop() call, so this is 4 consecutive loop iterations
    // with a matched detection, not a literal 1-second wall-clock timer.
    private static final int CONFIRM_FRAMES = 4;

    // How far (inches, camera-relative x/z) a detection can drift and still
    // count as the same ball we're already tracking.
    private static final double MATCH_DISTANCE_INCHES = 4.0;

    // How many consecutive missed frames are tolerated before we treat the
    // ball as lost - during SCANNING this resets the candidate, during
    // FOLLOWING this stops the robot and drops back to SCANNING.
    private static final int MAX_MISSED_FRAMES = 5;

    // ---- Proportional (visual-servo) drive gains ----
    // Power applied per inch (forward) or per degree (turn) of error.
    // These are starting points - tune on the actual robot, increasing
    // gradually until following feels responsive without oscillating.
    private static final double FORWARD_KP = 0.035;
    // No STRAFE_KP - centering is done by rotating to face the ball
    // (heading control), not by strafing. See followBall().
    private static final double TURN_KP = 0.02;
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double MAX_TURN_POWER = 0.4;

    // Errors smaller than this are treated as "close enough" so the robot
    // doesn't buzz/jitter right at the target distance/heading.
    private static final double DISTANCE_DEADBAND_INCHES = 1.0;
    private static final double ANGLE_DEADBAND_DEGREES = 2.0;

    // Only send telemetry.update() (network I/O) every N loops, same
    // reasoning as OFSB1Auto - addData() is cheap, update() is not.
    // Raised from 5 -> 10: telemetry.update() blocks on Driver Station
    // network I/O, and every stall it causes eats directly into loop()
    // timing, which shows up as choppy drive output. Sending less often
    // trades slightly staler telemetry for a more consistent loop rate.
    private static final int TELEMETRY_UPDATE_INTERVAL = 10;
    private int telemetryCounter = 0;

    private Follower follower;
    private OFSB1Subsystem robot;
    private OpenCvWebcam webcam;
    private OFSB1VisionProcessor visionProcessor;
    private volatile boolean cameraInitialized = false;

    private enum State { SCANNING, FOLLOWING }
    private State state = State.SCANNING;

    private int confirmCount = 0;
    private int missedFrames = 0;
    // Ball being tracked (confirmed or not). Carries over between SCANNING
    // and FOLLOWING so a brief loss-and-reacquire doesn't require starting
    // the match-distance check over from a blank slate.
    private OFSB1VisionProcessor.Detection candidate = null;

    // True once follower.startTeleopDrive() has been called. Guards against
    // ever calling setTeleOpDrive() before teleop-drive mode is actually
    // active - doing so throws a NullPointerException on Pose.getHeading()
    // deep inside VectorCalculator, because the follower's internal teleop
    // pose reference is null until startTeleopDrive() initializes it.
    private boolean teleopDriveStarted = false;

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
                // MJPEG instead of the default YUY2: most UVC webcams only
                // support their higher frame rates (often 30fps vs ~10-15fps)
                // in MJPEG mode at this resolution. More frames per second
                // means fresher, less-jumpy position readings feeding the
                // drive loop, which is part of what causes choppy motion.
                // NOTE: verify your EasyOpenCV version exposes this
                // startStreaming(width, height, rotation, streamFormat)
                // overload and that your specific webcam actually supports
                // MJPEG - if not, drop the last argument to fall back to
                // the previous default behavior.
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN,
                        OpenCvWebcam.StreamFormat.MJPEG);
                cameraInitialized = true;
            }

            @Override
            public void onError(int errorCode) {
                cameraInitialized = false;
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Status", "OFSB1 Follow Ball Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Camera Initialized", cameraInitialized);
        telemetry.addData("Ball Count", visionProcessor.getDetections().size());
        telemetry.update();
    }

    @Override
    public void start() {
        state = State.SCANNING;
        confirmCount = 0;
        missedFrames = 0;
        candidate = null;
        telemetryCounter = 0;

        // Must happen before any setTeleOpDrive() call - see the field
        // comment on teleopDriveStarted for why. Only needs to be called
        // once; teleop-drive mode stays active for the rest of the OpMode.
        follower.startTeleopDrive();
        teleopDriveStarted = true;
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case SCANNING:
                scanForBall();
                break;
            case FOLLOWING:
                followBall();
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Camera Initialized", cameraInitialized);
        if (candidate != null) {
            telemetry.addData("Ball X (in)", "%.1f", candidate.x);
            telemetry.addData("Ball Z (in)", "%.1f", candidate.z);
        }
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());

        telemetryCounter++;
        if (telemetryCounter >= TELEMETRY_UPDATE_INTERVAL) {
            telemetry.update();
            telemetryCounter = 0;
        }
    }

    /**
     * Same confirm-then-lock logic as OFSB1Auto's scanForBall(), but on
     * lock we switch to FOLLOWING instead of building one path and
     * stopping there - the ball is expected to keep moving.
     */
    private void scanForBall() {
        List<OFSB1VisionProcessor.Detection> balls = visionProcessor.getDetections();
        OFSB1VisionProcessor.Detection match = findMatch(balls);

        if (match == null) {
            missedFrames++;
            telemetry.addData("Status", "No match (" + missedFrames + "/" + MAX_MISSED_FRAMES + " missed)");
            if (missedFrames > MAX_MISSED_FRAMES) {
                confirmCount = 0;
                candidate = null;
            }
            return;
        }

        boolean sameCandidate = candidate != null
                && Math.hypot(match.x - candidate.x, match.z - candidate.z) <= MATCH_DISTANCE_INCHES;

        candidate = match;
        confirmCount = sameCandidate ? confirmCount + 1 : 1;
        missedFrames = 0;

        telemetry.addData("Status", "Confirming (" + confirmCount + "/" + CONFIRM_FRAMES + ")");

        if (confirmCount >= CONFIRM_FRAMES) {
            missedFrames = 0;
            state = State.FOLLOWING;
        }
    }

    /**
     * Continuous visual-servo follow: every loop, re-reads the ball's
     * camera-relative position and drives with power proportional to how
     * far off we are from DESIRED_DISTANCE_INCHES (forward/back) and dead
     * center (strafe). Recomputed from scratch every cycle - nothing here
     * commits to a fixed destination, so a moving ball just continuously
     * shifts the drive power instead of requiring a replan.
     */
    private void followBall() {
        List<OFSB1VisionProcessor.Detection> balls = visionProcessor.getDetections();
        OFSB1VisionProcessor.Detection match = findMatch(balls);

        if (match == null) {
            missedFrames++;
            telemetry.addData("Status", "Ball not seen (" + missedFrames + "/" + MAX_MISSED_FRAMES + ")");
            if (missedFrames > MAX_MISSED_FRAMES) {
                // Lost it - stop driving and go back to hunting rather than
                // continuing to drive blind on a stale last-known position.
                stopDrive();
                state = State.SCANNING;
                confirmCount = 0;
                candidate = null;
            }
            return;
        }

        missedFrames = 0;
        candidate = match;

        // Straight-line distance to the ball (not just camera-relative z),
        // since once the robot starts turning to face the ball, z alone no
        // longer represents "distance to close" - the true range does.
        double distanceToBall = Math.hypot(match.x, match.z);
        double distanceError = distanceToBall - DESIRED_DISTANCE_INCHES; // + = too far, - = too close

        // Angle from the camera's boresight (straight ahead) to the ball,
        // in degrees. + = ball is to the right, so the robot needs to turn
        // right (clockwise) to bring it onto the centerline.
        double angleToBallDegrees = Math.toDegrees(Math.atan2(match.x, match.z));

        double forwardPower = 0;
        if (Math.abs(distanceError) > DISTANCE_DEADBAND_INCHES) {
            forwardPower = clamp(distanceError * FORWARD_KP, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
        }

        double turnPower = 0;
        if (Math.abs(angleToBallDegrees) > ANGLE_DEADBAND_DEGREES) {
            turnPower = clamp(angleToBallDegrees * TURN_KP, -MAX_TURN_POWER, MAX_TURN_POWER);
        }

        // No strafe term - centering the ball on the camera's vertical
        // centerline is done entirely by rotating the robot (turnPower) so
        // it faces the ball, not by strafing sideways. forwardPower then
        // just closes/holds distance along whatever heading the robot is
        // currently facing.
        //
        // NOTE: the sign of turnPower here assumes positive turn power
        // rotates the robot clockwise (matching "+angle = ball is right,
        // turn right"). Verify this against your PedroPathing version's
        // setTeleOpDrive() turn convention on the actual robot - if the
        // robot turns away from the ball instead of toward it, negate
        // turnPower (or negate TURN_KP).
        setDrive(forwardPower, 0, turnPower);

        telemetry.addData("Status", "Following");
        telemetry.addData("Forward Power", "%.2f", forwardPower);
        telemetry.addData("Turn Power", "%.2f", turnPower);
        telemetry.addData("Angle To Ball (deg)", "%.1f", angleToBallDegrees);
    }

    private void stopDrive() {
        setDrive(0, 0, 0);
    }

    /**
     * Guarded wrapper around follower.setTeleOpDrive(). Only ever calls it
     * once teleopDriveStarted is true, which start() guarantees before
     * loop() ever runs. Prevents the NullPointerException on
     * Pose.getHeading() that occurs if setTeleOpDrive() is somehow reached
     * before startTeleopDrive() has initialized the follower's internal
     * teleop pose reference (e.g. if start() is ever refactored and the
     * ordering breaks).
     */
    private void setDrive(double forward, double strafe, double turn) {
        if (!teleopDriveStarted) {
            telemetry.addLine("WARNING: setDrive() called before startTeleopDrive() - ignoring");
            return;
        }
        follower.setTeleOpDrive(forward, strafe, turn, true);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Finds the detection this frame that best matches what we're
     * tracking. Prefers whichever ball is closest to the candidate's last
     * known position (so we don't jump to a different ball just because
     * it's momentarily nearer); falls back to the globally closest ball if
     * there's no candidate yet.
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
        return (bestDist <= MATCH_DISTANCE_INCHES) ? best : null;
    }

    @Override
    public void stop() {
        stopDrive();
        robot.stopAll();
        if (webcam != null) {
            webcam.stopStreaming();
        }
    }
}