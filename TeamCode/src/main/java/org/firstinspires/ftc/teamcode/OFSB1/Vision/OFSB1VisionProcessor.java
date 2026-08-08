package org.firstinspires.ftc.teamcode.OFSB1.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Detects BIOBUZZ-season POLLEN (yellow ~2.8-3in balls) using HSV color
 * thresholding + contour analysis, with a lenient circularity check so a
 * partially-shadowed or partially-occluded ball still counts.
 *
 * IMPORTANT: The HSV ranges below are starting points. Real lighting in your
 * gym/field WILL differ from these. Tune them with FTC Dashboard rather than
 * guessing blind.
 */
public class OFSB1VisionProcessor extends OpenCvPipeline {

    public enum ArtifactColor { YELLOW }

    public static class Detection {
        public ArtifactColor color;
        public Rect boundingBox;
        public Point center;
        public double area;
        public double circularity;

        // Camera-relative position, in inches:
        //   x: 0 = dead center, + right, - left
        //   y: 0 = camera height, + up, - down
        //   z: distance straight out from the camera lens
        public double x;
        public double y;
        public double z;
    }

    // ---- Physical / camera constants for distance estimation ----
    // Pollen ball real-world diameter (inches). BIOBUZZ spec is ~2.8-3in.
    private static final double BALL_DIAMETER_INCHES = 2.8;

    // Horizontal field of view of your webcam, in degrees, AT THE RESOLUTION
    // YOU ARE STREAMING (640x480 here). Replace with your webcam's actual spec.
    private static final double HORIZONTAL_FOV_DEGREES = 70.4;

    // Focal length in pixels - CALIBRATE THIS, don't trust the placeholder.
    // How to calibrate: place a ball at a known distance D (inches) from the
    // lens, read the detected boundingBox.width in pixels (call it W), then:
    //   FOCAL_LENGTH_PIXELS = (W * D) / BALL_DIAMETER_INCHES
    private static final double FOCAL_LENGTH_PIXELS = 700; // PLACEHOLDER - calibrate me

    private static final int FRAME_WIDTH = 640;
    private static final int FRAME_HEIGHT = 480;

    // ---- Tunable HSV thresholds (OpenCV HSV: H 0-179, S 0-255, V 0-255) ----
    // Pollen (yellow) - starting guess, TUNE THIS.
    private Scalar yellowLower = new Scalar(20, 100, 100);
    private Scalar yellowUpper = new Scalar(35, 255, 255);

    private static final double MIN_CONTOUR_AREA = 200; // px^2, coarse pre-filter before circle fit
    private static final double MIN_RADIUS_PX = 8;       // minimum enclosing-circle radius to count
    // Lenient on purpose - a ball with a shadow across part of it, or partly
    // cut off at the frame edge, should still pass. A full circle scores near
    // 1.0; this only rejects shapes that are clearly NOT round at all
    // (tape lines, jerseys, flat panels), not partially-occluded circles.
    private static final double MIN_CIRCULARITY = 0.45;

    // Working mats (allocated once, reused every frame - avoids GC churn)
    private final Mat blurred = new Mat();
    private final Mat hsvMat = new Mat();
    private final Mat yellowMask = new Mat();
    private final Mat morphKernel = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new org.opencv.core.Size(5, 5));

    // ---- Debug/diagnostic state, exposed for telemetry ----
    private volatile long frameCount = 0;
    private volatile long lastProcessTimeNs = 0;
    private volatile String lastError = null;
    private final List<Detection> detections = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        long start = System.nanoTime();
        try {
            synchronized (this) {
                detections.clear();
            }

            // Blur before thresholding - reduces mask noise/speckling that
            // otherwise fragments a single ball into multiple small blobs.
            Imgproc.GaussianBlur(input, blurred, new org.opencv.core.Size(7, 7), 0);
            Imgproc.cvtColor(blurred, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, yellowLower, yellowUpper, yellowMask);

            // OPEN (erode->dilate) clears small noise specks.
            // CLOSE (dilate->erode) fills small gaps/holes back in - this is
            // the step that was missing before, and is likely why a ball
            // with a glare highlight or partial shadow was being detected
            // as two smaller blobs (or dropped below MIN_CONTOUR_AREA)
            // instead of one solid one.
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, morphKernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, morphKernel);

            findAndDraw(input, yellowMask, ArtifactColor.YELLOW, new Scalar(0, 255, 255));

            lastError = null;
        } catch (Exception e) {
            // EasyOpenCV silently swallows exceptions thrown from processFrame
            // and just freezes the last good frame - so capture it ourselves.
            lastError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }

        frameCount++;
        lastProcessTimeNs = System.nanoTime() - start;
        return input;
    }

    private void findAndDraw(Mat input, Mat mask, ArtifactColor color, Scalar drawColor) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area < MIN_CONTOUR_AREA) continue;

            double perimeter = Imgproc.arcLength(new MatOfPoint2f(c.toArray()), true);
            if (perimeter <= 0) continue;
            double circularity = (4 * Math.PI * area) / (perimeter * perimeter);
            if (circularity < MIN_CIRCULARITY) continue; // not round enough - skip (e.g. tape, jersey)

            // Fit a circle to the contour rather than a bounding box - a
            // tighter, more accurate size/center estimate for round objects,
            // especially when part of the ball is shadowed or occluded.
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());
            Point circleCenter = new Point();
            float[] circleRadius = new float[1];
            Imgproc.minEnclosingCircle(contour2f, circleCenter, circleRadius);
            contour2f.release();

            if (circleRadius[0] < MIN_RADIUS_PX) continue;

            Rect box = Imgproc.boundingRect(c);
            Detection d = new Detection();
            d.color = color;
            d.boundingBox = box;
            d.center = circleCenter;
            d.area = area;
            d.circularity = circularity;

            // Z: distance from camera to ball, using known ball size vs apparent pixel diameter
            double pixelDiameter = circleRadius[0] * 2.0;
            d.z = (BALL_DIAMETER_INCHES * FOCAL_LENGTH_PIXELS) / pixelDiameter;

            // X: horizontal offset in inches, via angle-off-centerline * distance
            double pixelsPerDegreeH = FRAME_WIDTH / HORIZONTAL_FOV_DEGREES;
            double angleXDegrees = (d.center.x - FRAME_WIDTH / 2.0) / pixelsPerDegreeH;
            d.x = d.z * Math.tan(Math.toRadians(angleXDegrees));

            // Y: vertical offset in inches (same angular approach, using vertical FOV
            // derived from aspect ratio - NOT true height off the ground)
            double verticalFovDegrees = HORIZONTAL_FOV_DEGREES * FRAME_HEIGHT / FRAME_WIDTH;
            double pixelsPerDegreeV = FRAME_HEIGHT / verticalFovDegrees;
            double angleYDegrees = (d.center.y - FRAME_HEIGHT / 2.0) / pixelsPerDegreeV;
            d.y = d.z * Math.tan(Math.toRadians(angleYDegrees));

            synchronized (this) {
                detections.add(d);
            }

            Imgproc.circle(input, circleCenter, (int) circleRadius[0], drawColor, 3);
            Imgproc.circle(input, d.center, 4, drawColor, -1);
            Imgproc.putText(input,
                    String.format("Z=%.1f\" c=%.2f", d.z, d.circularity),
                    new Point(box.x, Math.max(box.y - 8, 12)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, drawColor, 2);
        }
        hierarchy.release();
    }

    // ---------------- Public API used by TeleOp/Auto ----------------

    /** Snapshot of current detections. Safe to call from loop(). */
    public synchronized List<Detection> getDetections() {
        return new ArrayList<>(detections);
    }

    /** Frame counter - if this isn't increasing, the pipeline isn't running. */
    public long getFrameCount() {
        return frameCount;
    }

    public double getLastProcessTimeMs() {
        return lastProcessTimeNs / 1_000_000.0;
    }

    /** Non-null if the last processFrame() call threw - check this first when debugging. */
    public String getLastError() {
        return lastError;
    }

    public void setYellowRange(Scalar lower, Scalar upper) {
        this.yellowLower = lower;
        this.yellowUpper = upper;
    }
}