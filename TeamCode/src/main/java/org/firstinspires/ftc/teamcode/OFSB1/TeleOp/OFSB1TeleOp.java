package org.firstinspires.ftc.teamcode.OFSB1.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
 * TeleOp for Off Season Bot 1 (OFSB1).
 * Uses PedroPathing for driving.
 */
@TeleOp(name = "OFSB1 TeleOp", group = "OFSB1")
public class OFSB1TeleOp extends OpMode {

    private Follower follower;
    private OFSB1Subsystem robot;
    private OpenCvWebcam webcam;
    private OFSB1VisionProcessor visionProcessor;
    private volatile boolean cameraInitialized = false;

    @Override
    public void init() {
        // Initialize PedroPathing Follower using OFSB1-specific constants
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Initialize robot subsystems
        robot = new OFSB1Subsystem(hardwareMap);

        // Initialize Vision
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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
            }
        });

        telemetry.addData("Status", "OFSB1 Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        addVisionTelemetry();
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        // Driving control
        // Strafing lives on the analog triggers (L2/R2 on PS4, LT/RT on
        // Xbox/Logitech): L2 strafes left, R2 strafes right, and how far
        // you pull controls the speed. Matches the old stick convention
        // where positive strafe = left.
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_trigger - gamepad1.right_trigger;
        double turn = -gamepad1.right_stick_x;
        follower.setTeleOpDrive(forward, strafe, turn, true);

        // Add robot subsystem controls here
        // if (gamepad1.a) robot.doSomething();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());

        // ---- Pollen (yellow ball) vision telemetry ----
        addVisionTelemetry();

        telemetry.update();
    }

    /**
     * Vision telemetry shared by init_loop() and loop(): per-ball position
     * data plus pipeline health/diagnostics from the vision processor.
     */
    private void addVisionTelemetry() {
        List<OFSB1VisionProcessor.Detection> balls = visionProcessor.getDetections();

        telemetry.addData("Camera Initialized", cameraInitialized);
        telemetry.addData("Number of balls detected", balls.size());

        if (balls.isEmpty()) {
            telemetry.addLine("No balls detected - check lighting/camera aim");
        } else {
            for (int i = 0; i < balls.size(); i++) {
                OFSB1VisionProcessor.Detection ball = balls.get(i);
                telemetry.addData("Ball #" + (i + 1),
                        "X: %.1f in, Y: %.1f in, Z: %.1f in, r: %.0f px, score: %.2f",
                        ball.x, ball.y, ball.z, ball.radiusPx, ball.circularity);
            }
        }

        // ---- Pipeline health ----
        // Frame count not increasing = pipeline stalled.
        telemetry.addData("Vision Frames", visionProcessor.getFrameCount());
        // Watch this while balls overlap - if it climbs too high, the Hough
        // splitter is costing too much per frame.
        telemetry.addData("Vision Frame Time (ms)", "%.1f", visionProcessor.getLastProcessTimeMs());
        // 0 on a clean scene; pinned at max = HSV range matching non-ball stuff.
        telemetry.addData("Hough Splits Last Frame", visionProcessor.getHoughRunsLastFrame());
        String visionError = visionProcessor.getLastError();
        if (visionError != null) {
            telemetry.addData("VISION ERROR", visionError);
        }
    }

    @Override
    public void stop() {
        robot.stopAll();
        if (webcam != null) {
            webcam.stopStreaming();
        }
    }
}