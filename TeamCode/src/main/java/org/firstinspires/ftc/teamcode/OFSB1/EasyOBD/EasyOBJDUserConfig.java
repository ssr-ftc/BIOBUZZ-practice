/*
 * Copyright (c) 2026 Akash Vijay Aradhya
 *
 * SPDX-License-Identifier: MIT
 *
 * EasyOBJD - FTC EasyOpenCV object detection
 * https://github.com/IamAki123/EasyOBJD
 */
package org.firstinspires.ftc.teamcode.OFSB1.EasyOBD;

import org.firstinspires.ftc.easyobjd.EasyOBJDConfig;
import org.firstinspires.ftc.easyobjd.OverlayMode;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * Copy this file into TeamCode and edit the numbers here.
 * You do not need to change anything inside the EasyOBJD library.
 *
 * <p>After you run {@code EasyOBJD Tuner} (D-pad up = wider, down = tighter),
 * copy the HSV values from telemetry back into the fields below.
 *
 * <p>OpenCV 8-bit HSV: H is 0–179, S and V are 0–255.
 */
public final class EasyOBJDUserConfig {
    private EasyOBJDUserConfig() {}

    /**
     * Must match the webcam name in the Robot Controller configuration
     * (Configure Robot → Webcam). Common names: {@code "Webcam 1"}, {@code "Webcam 2"}.
     */
    public static String WEBCAM_NAME = "Webcam 1";

    public static int STREAM_WIDTH = 640;
    public static int STREAM_HEIGHT = 480;
    public static OpenCvCameraRotation STREAM_ROTATION = OpenCvCameraRotation.UPRIGHT;

    /**
     * Process width in pixels. 320 is faster on a Control Hub; 640 is better
     * for distant pieces. Height follows the stream aspect. 0 = stream size.
     */
    public static int PROCESS_WIDTH = 320;

    // ---- Game piece ----
    /** Official piece diameter, inches (size-based range + floor-plane height). */
    public static double BALL_DIAMETER_INCHES = 2.8;
    /**
     * Smallest object accepted, inches (hole / far-object cutoff).
     * Keep a bit larger than {@link #BALL_DIAMETER_INCHES}.
     */
    public static double MIN_BALL_DIAMETER_INCHES = 3.0;

    // ---- HSV (defaults = yellow pollen). Change for other colors / seasons. ----
    public static double H_LOW = 21;
    public static double S_LOW = 95;
    public static double V_LOW = 85;
    public static double H_HIGH = 35;
    public static double S_HIGH = 255;
    public static double V_HIGH = 255;

    // ---- Camera mount: run EasyOBJD Range Test and paste its numbers here. ----
    // These came from one team's robot (#23918). Yours WILL be different, and
    // distances are wrong until you replace them.
    /** Floor to the center of the lens. */
    public static double CAMERA_HEIGHT_INCHES = 18.0;
    /** How far the lens sits behind the front of the robot. */
    public static double CAMERA_BEHIND_FRONT_INCHES = 6.5;
    public static double CAMERA_TILT_DEGREES = 15.58;
    public static double HORIZONTAL_FOV_DEGREES = 70.4;
    /** 0 = derive from FOV. Range Test solves this from two taped distances. */
    public static double FOCAL_LENGTH_PIXELS_AT_640 = 710.4;

    public static double MAX_RANGE_INCHES = 60.0;
    public static boolean ADAPTIVE_LIGHTING = true;

    /** Occupied yellow cells that touch become one cluster. */
    public static int GRID_ROWS = 12;
    public static int GRID_COLS = 12;
    /** FULL draws the 12×12 regions, balls, and cluster markers. */
    public static OverlayMode OVERLAY = OverlayMode.FULL;

    /** Builds the library config from the fields above. */
    public static EasyOBJDConfig create() {
        return EasyOBJDConfig.builder()
                .camera(CAMERA_HEIGHT_INCHES, CAMERA_TILT_DEGREES, HORIZONTAL_FOV_DEGREES)
                .focalLengthPixelsAt640(FOCAL_LENGTH_PIXELS_AT_640)
                .hsv(H_LOW, S_LOW, V_LOW, H_HIGH, S_HIGH, V_HIGH)
                .ballDiameterInches(BALL_DIAMETER_INCHES)
                .minBallDiameterInches(MIN_BALL_DIAMETER_INCHES)
                .maxRangeInches(MAX_RANGE_INCHES)
                .processWidth(PROCESS_WIDTH)
                .adaptiveLighting(ADAPTIVE_LIGHTING)
                .grid(GRID_ROWS, GRID_COLS)
                .overlayMode(OVERLAY)
                .build();
    }
}
