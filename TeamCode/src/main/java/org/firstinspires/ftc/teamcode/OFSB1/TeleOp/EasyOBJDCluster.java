/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.teamcode.OFSB1.TeleOp;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.easyobjd.ClusterInfo;
import org.firstinspires.ftc.easyobjd.EasyOBJD;
import org.firstinspires.ftc.easyobjd.EasyOBJDConfig;
import org.firstinspires.ftc.easyobjd.EasyOBJDPipeline;
import org.firstinspires.ftc.easyobjd.LocalizationMath;
import org.firstinspires.ftc.easyobjd.OverlayMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

/**
 * Tape-measure range check and camera-mount calibration.
 *
 * <p>Put one ball straight ahead of the camera. Tape from the front of the
 * robot to the near edge of the ball, along the floor. Set that as the target
 * with the D-pad <b>before</b> pressing A, and
 * the OpMode solves the camera tilt that makes that ball read correctly. A
 * second point at a different distance (8+ in apart) also solves focal
 * length, which fixes a wrong FOV. Copy the telemetry values into
 * {@code EasyOBJDUserConfig}.</p>
 *
 * <p>Gamepad 1: D-pad up/down target ±1 in, left/right ±6 in; A capture;
 * B reset; bumpers HSV tighter/wider; X overlay.</p>
 *
 * <p>Calibration math is inline so this runs against JitPack 1.0.2.</p>
 */
@SuppressWarnings("unused")
@TeleOp(name = "EasyOBJD Range Test", group = "EasyOBJD")
public class EasyOBJDCluster extends OpMode {
    public static final String WEBCAM_NAME = "Webcam 1";
    public static final int STREAM_WIDTH = 640;
    public static final int STREAM_HEIGHT = 480;

    // ---- Starting mount. Height is floor to the center of the lens. ----
    public static double CAMERA_HEIGHT_INCHES = 18.0;
    /** How far the lens sits behind the front of the robot. */
    public static double CAMERA_BEHIND_FRONT_INCHES = 6.5;
    public static double CAMERA_TILT_DEGREES = 25.0;
    public static double HORIZONTAL_FOV_DEGREES = 70.4;
    /** 0 = derive from FOV. */
    public static double FOCAL_LENGTH_PIXELS_AT_640 = 0;
    public static double BALL_DIAMETER_INCHES = 2.8;

    private static final int FRAMES_PER_CAPTURE = 15;
    private static final double MIN_POINT_SPACING_INCHES = 8.0;

    private OpenCvWebcam webcam;
    private EasyOBJDPipeline pipeline;
    private volatile boolean cameraInitialized = false;

    /** Front of robot to near edge of the ball. */
    private double targetInches = 24.0;
    private double tiltDegrees = CAMERA_TILT_DEGREES;
    private double focalAt640 = FOCAL_LENGTH_PIXELS_AT_640;
    private boolean focalSolved = false;

    /** Each entry: {image row of the ball, taped forward inches}. */
    private final List<double[]> points = new ArrayList<double[]>();
    private int captureFramesLeft = 0;
    private double captureSumY = 0;
    private int captureCount = 0;
    private long lastFrame = -1;
    private String status = "Place one ball, set target, press A";

    @SuppressLint("DiscouragedApi")
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        pipeline = EasyOBJD.createPipeline();
        EasyOBJDConfig cfg = pipeline.getConfig();
        cfg.cameraHeightInches = CAMERA_HEIGHT_INCHES;
        cfg.ballDiameterInches = BALL_DIAMETER_INCHES;
        cfg.horizontalFovDegrees = HORIZONTAL_FOV_DEGREES;
        cfg.overlayMode = OverlayMode.FULL;
        applyMount();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT,
                        OpenCvWebcam.StreamFormat.MJPEG);
                cameraInitialized = true;
            }

            @Override
            public void onError(int errorCode) {
                cameraInitialized = false;
                telemetry.addData("Camera Error", errorCode);
            }
        });
    }

    @Override
    public void init_loop() {
        tick();
    }

    @Override
    public void loop() {
        tick();
    }

    private void tick() {
        handleInput();
        ClusterInfo nearest = nearestCluster(pipeline.getClusters());
        collectCapture(nearest);

        if (!cameraInitialized) {
            telemetry.addLine("Camera starting...");
        }
        telemetry.addLine("Tape robot front -> near edge of ball");
        telemetry.addLine("Set target to the tape BEFORE pressing A");
        telemetry.addLine("D-pad target, A capture, B reset, bumpers HSV, X overlay");
        telemetry.addData("Status", status);
        telemetry.addData("Target (from front)", "%.0f in", targetInches);
        telemetry.addData("Points", points.size());

        if (nearest == null) {
            telemetry.addLine("No cluster - widen HSV with right bumper");
        } else {
            double startFront = fromFront(startMountForward(nearest));
            double nowFront = fromFront(nearest.y);
            telemetry.addData("Method", nearest.localization);
            telemetry.addData("From front, start mount", "%.1f in (err %+.1f)",
                    startFront, startFront - targetInches);
            telemetry.addData(points.isEmpty() ? "From front, no calibration yet" : "From front, calibrated",
                    "%.1f in (err %+.1f)", nowFront, nowFront - targetInches);
            telemetry.addData("Lens Y / X", "%.1f / %.1f in", nearest.y, nearest.x);
            telemetry.addData("Balls / clusters", "%d / %d",
                    pipeline.getBallCount(), pipeline.getClusterCount());
        }

        telemetry.addLine("--- Paste into EasyOBJDUserConfig ---");
        telemetry.addData("CAMERA_HEIGHT_INCHES", "%.1f", CAMERA_HEIGHT_INCHES);
        telemetry.addData("CAMERA_TILT_DEGREES", "%.2f", tiltDegrees);
        if (focalSolved) {
            telemetry.addData("FOCAL_LENGTH_PIXELS_AT_640", "%.1f", focalAt640);
        } else {
            telemetry.addData("FOCAL_LENGTH_PIXELS_AT_640", "%.1f (add a 2nd point to solve)",
                    focalAt640);
        }
        telemetry.update();
    }

    private void handleInput() {
        if (gamepad1.dpadUpWasPressed()) {
            targetInches += 1;
        }
        if (gamepad1.dpadDownWasPressed()) {
            targetInches = Math.max(6, targetInches - 1);
        }
        if (gamepad1.dpadRightWasPressed()) {
            targetInches += 6;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            targetInches = Math.max(6, targetInches - 6);
        }
        if (gamepad1.rightBumperWasPressed()) {
            pipeline.adjustHsvRange(1);
        }
        if (gamepad1.leftBumperWasPressed()) {
            pipeline.adjustHsvRange(-1);
        }
        if (gamepad1.xWasPressed()) {
            pipeline.cycleOverlayMode();
        }
        if (gamepad1.aWasPressed() && captureFramesLeft == 0) {
            captureFramesLeft = FRAMES_PER_CAPTURE;
            captureSumY = 0;
            captureCount = 0;
            status = "Capturing - hold still";
        }
        if (gamepad1.bWasPressed()) {
            points.clear();
            captureFramesLeft = 0;
            tiltDegrees = CAMERA_TILT_DEGREES;
            focalAt640 = FOCAL_LENGTH_PIXELS_AT_640;
            focalSolved = false;
            applyMount();
            status = "Reset to starting mount";
        }
    }

    /** Averages the nearest cluster's image row over several new frames. */
    private void collectCapture(ClusterInfo nearest) {
        if (captureFramesLeft == 0) {
            return;
        }
        long frame = pipeline.getFrameCount();
        if (frame == lastFrame) {
            return;
        }
        lastFrame = frame;
        captureFramesLeft--;
        if (nearest != null) {
            captureSumY += nearest.centerPx.y;
            captureCount++;
        }
        if (captureFramesLeft > 0) {
            return;
        }
        if (captureCount < FRAMES_PER_CAPTURE / 2) {
            status = "Capture failed - ball not seen";
            return;
        }
        points.add(new double[] { captureSumY / captureCount, lensForward(targetInches) });
        solve();
    }

    private void solve() {
        int w = processWidth();
        int h = processHeight(w);

        double[] a = null;
        double[] b = null;
        for (int i = 0; i < points.size(); i++) {
            for (int j = i + 1; j < points.size(); j++) {
                double spacing = Math.abs(points.get(i)[1] - points.get(j)[1]);
                if (spacing >= MIN_POINT_SPACING_INCHES
                        && (a == null || spacing > Math.abs(a[1] - b[1]))) {
                    a = points.get(i);
                    b = points.get(j);
                }
            }
        }
        focalSolved = false;
        if (a != null) {
            double f640 = focalForTwoFloorPoints(a[0], a[1], b[0], b[1], w, h);
            double fov = Double.isNaN(f640) ? Double.NaN
                    : Math.toDegrees(2.0 * Math.atan(320.0 / f640));
            if (fov >= 30 && fov <= 120) {
                focalAt640 = f640;
                focalSolved = true;
            }
        }

        double focalPx = LocalizationMath.focalPx(focalAt640, HORIZONTAL_FOV_DEGREES, w);
        double sum = 0;
        for (double[] p : points) {
            sum += tiltForFloorPoint(p[0], h, focalPx, p[1]);
        }
        tiltDegrees = sum / points.size();
        applyMount();
        String last = String.format("last point %.0f in from front",
                fromFront(points.get(points.size() - 1)[1]));
        status = focalSolved
                ? "Solved tilt + focal from " + points.size() + " points, " + last
                : "Solved tilt from " + points.size() + " point(s), " + last;
    }

    private void applyMount() {
        EasyOBJDConfig cfg = pipeline.getConfig();
        cfg.cameraTiltDegrees = tiltDegrees;
        cfg.focalLengthPixelsAt640 = focalAt640;
    }

    private double tiltForFloorPoint(double pixelY, int frameHeight, double focalPx, double forward) {
        double drop = CAMERA_HEIGHT_INCHES - BALL_DIAMETER_INCHES / 2.0;
        double aboveAxis = Math.atan2(frameHeight / 2.0 - pixelY, focalPx);
        return Math.toDegrees(aboveAxis + Math.atan2(drop, forward));
    }

    private double focalForTwoFloorPoints(double py1, double z1, double py2, double z2,
                                          int frameWidth, int frameHeight) {
        double a1 = frameHeight / 2.0 - py1;
        double a2 = frameHeight / 2.0 - py2;
        if (Math.abs(a1 - a2) < 0.05 * frameHeight) {
            return Double.NaN;
        }
        double drop = CAMERA_HEIGHT_INCHES - BALL_DIAMETER_INCHES / 2.0;
        double target = Math.atan2(drop, z2) - Math.atan2(drop, z1);
        double lo = Math.max(0.3 * frameWidth, Math.sqrt(Math.max(0, a1 * a2)) + 1.0);
        double hi = 5.0 * frameWidth;
        double gLo = Math.atan2(a1, lo) - Math.atan2(a2, lo) - target;
        double gHi = Math.atan2(a1, hi) - Math.atan2(a2, hi) - target;
        if (gLo * gHi > 0) {
            return Double.NaN;
        }
        for (int i = 0; i < 60; i++) {
            double mid = 0.5 * (lo + hi);
            double gMid = Math.atan2(a1, mid) - Math.atan2(a2, mid) - target;
            if (gLo * gMid <= 0) {
                hi = mid;
            } else {
                lo = mid;
                gLo = gMid;
            }
        }
        return 0.5 * (lo + hi) * (640.0 / frameWidth);
    }

    /** Robot-front-to-near-edge tape distance → lens-to-ball-center forward inches. */
    private static double lensForward(double fromFrontInches) {
        return fromFrontInches + CAMERA_BEHIND_FRONT_INCHES + BALL_DIAMETER_INCHES / 2.0;
    }

    private static double fromFront(double lensForwardInches) {
        return lensForwardInches - CAMERA_BEHIND_FRONT_INCHES - BALL_DIAMETER_INCHES / 2.0;
    }

    /** Forward inches for this cluster using the untouched starting mount. */
    private double startMountForward(ClusterInfo cluster) {
        int w = processWidth();
        int h = processHeight(w);
        double[] floor = LocalizationMath.pixelToFloor(
                cluster.centerPx.x, cluster.centerPx.y, w, h,
                CAMERA_HEIGHT_INCHES, BALL_DIAMETER_INCHES,
                FOCAL_LENGTH_PIXELS_AT_640, HORIZONTAL_FOV_DEGREES, CAMERA_TILT_DEGREES);
        return floor == null ? Double.NaN : floor[2];
    }

    /** Lowest cluster in the image is the closest one on the floor. */
    private static ClusterInfo nearestCluster(List<ClusterInfo> clusters) {
        ClusterInfo best = null;
        for (ClusterInfo c : clusters) {
            if (best == null || c.centerPx.y > best.centerPx.y) {
                best = c;
            }
        }
        return best;
    }

    private int processWidth() {
        EasyOBJDConfig cfg = pipeline.getConfig();
        if (cfg.processWidth > 0) {
            return cfg.processWidth;
        }
        if (cfg.processScale < 0.999) {
            return (int) Math.round(STREAM_WIDTH * cfg.processScale);
        }
        return STREAM_WIDTH;
    }

    private static int processHeight(int processWidth) {
        return (int) Math.round(STREAM_HEIGHT * processWidth / (double) STREAM_WIDTH);
    }

    @Override
    public void stop() {
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }
}
