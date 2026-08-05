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
 * Scans for the closest yellow Pollen ball via OFSB1VisionProcessor, converts
 * its camera-relative x/z offset into a field-relative target pose, builds a
 * PedroPathing Path to a point TARGET_DISTANCE_INCHES short of the ball along
 * the line-of-sight, then follows that path.
 */
@Autonomous(name = "OFSB1 Auto", group = "OFSB1")
public class OFSB1Auto extends OpMode {

    private static final double TARGET_DISTANCE_INCHES = 5.0;
    // How many consecutive frames the ball must be seen before we trust it
    // and commit to a path - filters out one-frame noise blobs.
    private static final int CONFIRM_FRAMES = 5;

    private Follower follower;
    private OFSB1Subsystem robot;
    private OpenCvWebcam webcam;
    private OFSB1VisionProcessor visionProcessor;
    private volatile boolean cameraInitialized = false;

    private enum State { SCANNING, DRIVING, DONE }
    private State state = State.SCANNING;
    private int confirmCount = 0;

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
    public void start() {
        state = State.SCANNING;
        confirmCount = 0;
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
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    private void scanForBall() {
        List<OFSB1VisionProcessor.Detection> balls = visionProcessor.getDetections();

        if (balls.isEmpty()) {
            confirmCount = 0;
            telemetry.addData("Status", "Scanning - no ball seen");
            return;
        }

        // Closest ball = smallest z (actual distance from camera)
        OFSB1VisionProcessor.Detection target = balls.get(0);
        for (OFSB1VisionProcessor.Detection b : balls) {
            if (b.z < target.z) target = b;
        }

        confirmCount++;
        telemetry.addData("Status", "Ball seen, confirming (" + confirmCount + "/" + CONFIRM_FRAMES + ")");
        telemetry.addData("Ball X (in)", "%.1f", target.x);
        telemetry.addData("Ball Z (in)", "%.1f", target.z);

        if (confirmCount < CONFIRM_FRAMES) {
            return; // keep scanning until confirmed stable
        }

        buildAndFollowPathToBall(target);
    }

    private void buildAndFollowPathToBall(OFSB1VisionProcessor.Detection target) {
        Pose robotPose = follower.getPose();

        // Convert camera-relative offset into a field-relative bearing/distance.
        // x: lateral offset (in), z: forward distance (in), both camera-relative.
        double angleOffsetRadians = Math.atan2(target.x, target.z);
        double distanceToBall = Math.hypot(target.x, target.z);
        double driveDistance = distanceToBall - TARGET_DISTANCE_INCHES;

        double fieldAngle = robotPose.getHeading() + angleOffsetRadians;

        // Both targetX/targetY are computed in the SAME field frame as
        // robotPose - do not negate one axis without negating the other,
        // or start/end points end up in mismatched coordinate frames.
        double targetX = robotPose.getX() + driveDistance * Math.cos(fieldAngle);
        double targetY = robotPose.getY() + driveDistance * Math.sin(fieldAngle);
        Pose targetPose = new Pose(targetX, targetY, fieldAngle);

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