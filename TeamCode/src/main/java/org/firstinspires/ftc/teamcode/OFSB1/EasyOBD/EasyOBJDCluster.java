/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.teamcode.OFSB1.EasyOBD;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.easyobjd.ClusterInfo;
import org.firstinspires.ftc.easyobjd.EasyOBJD;
import org.firstinspires.ftc.easyobjd.EasyOBJDConfig;
import org.firstinspires.ftc.easyobjd.EasyOBJDPipeline;
import org.firstinspires.ftc.easyobjd.OverlayMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

/**
 * Copy this file and {@code EasyOBJDUserConfig} into TeamCode. Preview is the
 * 12×12 cluster grid (FULL overlay). Touching yellow cells become one cluster;
 * telemetry is occupied cells and each cluster's camera X/Y (inches).
 *
 * <p>Camera height, tilt, and focal length come from {@code EasyOBJDUserConfig}.
 * Inches are only right after you run <b>EasyOBJD Range Test</b> on your robot
 * and paste its numbers there.</p>
 *
 * <p>Gamepad 1: D-pad up/down HSV, left/right MASK / GRID / FULL.</p>
 *
 * <p>TeamCode will not resolve {@code org.firstinspires.ftc.easyobjd} until
 * JitPack is a repository and TeamCode depends on the library:</p>
 * <pre>
 * maven { url = 'https://jitpack.io' }
 * implementation 'org.openftc:easyopencv:1.7.3'
 * implementation 'com.github.IamAki123:EasyOBJD:1.0.2'
 * </pre>
 * Then File → Sync Project with Gradle Files.
 */
//noinspection SpellCheckingInspection
@SuppressWarnings("unused")
@TeleOp(name = "EasyOBJD Cluster", group = "EasyOBJD")
public class EasyOBJDCluster extends OpMode {
    public static final String WEBCAM_NAME = "Webcam 1";

    private static final OverlayMode[] OVERLAYS = {
            OverlayMode.MASK, OverlayMode.GRID, OverlayMode.FULL
    };

    private OpenCvWebcam webcam;
    private EasyOBJDPipeline pipeline;
    private volatile boolean cameraInitialized = false;

    @SuppressLint("DiscouragedApi")
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        pipeline = EasyOBJD.createPipeline();
        EasyOBJDConfig cfg = pipeline.getConfig();
        cfg.cameraHeightInches = EasyOBJDUserConfig.CAMERA_HEIGHT_INCHES;
        cfg.cameraTiltDegrees = EasyOBJDUserConfig.CAMERA_TILT_DEGREES;
        cfg.horizontalFovDegrees = EasyOBJDUserConfig.HORIZONTAL_FOV_DEGREES;
        cfg.focalLengthPixelsAt640 = EasyOBJDUserConfig.FOCAL_LENGTH_PIXELS_AT_640;
        cfg.ballDiameterInches = EasyOBJDUserConfig.BALL_DIAMETER_INCHES;
        cfg.overlayMode = OverlayMode.FULL;
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT,
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
        if (gamepad1.dpadUpWasPressed()) {
            pipeline.adjustHsvRange(1);
        }
        if (gamepad1.dpadDownWasPressed()) {
            pipeline.adjustHsvRange(-1);
        }
        if (gamepad1.dpadRightWasPressed()) {
            cycleOverlay(1);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            cycleOverlay(-1);
        }

        if (!cameraInitialized) {
            telemetry.addLine("Camera starting...");
        }
        telemetry.addLine("12x12 grid: touching yellow cells = one cluster");
        telemetry.addLine("D-pad UP/DOWN HSV, LEFT/RIGHT overlay");
        telemetry.addData("Overlay", pipeline.getConfig().overlayMode);
        telemetry.addData("Grid", "%d x %d",
                pipeline.getConfig().gridRows, pipeline.getConfig().gridCols);
        telemetry.addData("HSV lower (H,S,V)", "%.0f, %.0f, %.0f",
                pipeline.getConfig().hsvLower.val[0],
                pipeline.getConfig().hsvLower.val[1],
                pipeline.getConfig().hsvLower.val[2]);
        telemetry.addData("HSV upper (H,S,V)", "%.0f, %.0f, %.0f",
                pipeline.getConfig().hsvUpper.val[0],
                pipeline.getConfig().hsvUpper.val[1],
                pipeline.getConfig().hsvUpper.val[2]);
        telemetry.addData("Occupied cells", pipeline.getOccupiedCount());
        telemetry.addData("Clusters", pipeline.getClusterCount());
        telemetry.addData("Balls", pipeline.getBallCount());

        List<ClusterInfo> clusters = pipeline.getClusters();
        for (ClusterInfo cluster : clusters) {
            telemetry.addLine("Cluster #" + cluster.id);
            telemetry.addData("#" + cluster.id + " cells", cluster.cells.size());
            telemetry.addData("#" + cluster.id + " X", "%.1f in", cluster.x);
            telemetry.addData("#" + cluster.id + " Y", "%.1f in", cluster.y);
            telemetry.addData("#" + cluster.id + " from robot front", "%.1f in",
                    cluster.y - EasyOBJDUserConfig.CAMERA_BEHIND_FRONT_INCHES
                            - EasyOBJDUserConfig.BALL_DIAMETER_INCHES / 2.0);
        }
        telemetry.update();
    }

    private void cycleOverlay(int step) {
        OverlayMode current = pipeline.getConfig().overlayMode;
        int idx = 0;
        for (int i = 0; i < OVERLAYS.length; i++) {
            if (OVERLAYS[i] == current) {
                idx = i;
                break;
            }
        }
        pipeline.getConfig().overlayMode = OVERLAYS[Math.floorMod(idx + step, OVERLAYS.length)];
    }

    @Override
    public void stop() {
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }
}
