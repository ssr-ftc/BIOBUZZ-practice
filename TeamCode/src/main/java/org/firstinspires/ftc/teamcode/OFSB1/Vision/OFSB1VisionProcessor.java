package org.firstinspires.ftc.teamcode.OFSB1.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class OFSB1VisionProcessor extends OpenCvPipeline {

    public enum ArtifactColor { YELLOW }

    public static class Detection {
        public ArtifactColor color;
        public Rect boundingBox;
        public Point center;
        public double area;
        // Classic circularity score: 4*pi*A / P^2. 1.0 = perfect circle.
        public double circularity;
        // Fitted circle radius in pixels (what the Z estimate is based on).
        public double radiusPx;

        // Camera-relative position, in inches:
        //   x: 0 = dead center, + right, - left
        //   y: 0 = camera height, + up, - down
        //   z: distance straight out from the camera lens
        public double x;
        public double y;
        public double z;
    }

    // ---- Physical / camera constants for distance estimation ----
    // Real-world ball diameter (inches). Official AndyMark am-5851 Pollen
    // spec: 2.8in +/- 0.1in. THIS CHANGED FROM 5.0 - if you already
    // calibrated FOCAL_LENGTH_PIXELS assuming a 5in ball, redo that
    // calibration against the real 2.8in ball or your Z readings will be
    // wrong by roughly a factor of 5/2.8 (~1.79x too far).
    private static final double BALL_DIAMETER_INCHES = 2.8;

    // Horizontal field of view of your webcam, in degrees, AT THE RESOLUTION
    // YOU ARE STREAMING (640x480 here). Replace with your webcam's actual spec.
    private static final double HORIZONTAL_FOV_DEGREES = 70.4;

    // Focal length in pixels - CALIBRATE THIS, don't trust the placeholder.
    // How to calibrate: place a ball at a known distance D (inches) from the
    // lens, read the detected radiusPx (call it R), then:
    //   FOCAL_LENGTH_PIXELS = (2 * R * D) / BALL_DIAMETER_INCHES
    // (Redo this if you switch ball types - it bakes in the 5in diameter.)
    private static final double FOCAL_LENGTH_PIXELS = 700; // PLACEHOLDER - calibrate me

    private static final int FRAME_WIDTH = 640;
    private static final int FRAME_HEIGHT = 480;

    // ---- Camera mounting (inches / degrees) ----
    // Lens height above the floor.
    private static final double CAMERA_HEIGHT_INCHES = 19.0;
    // Downward pitch of the camera. 0 = aimed perfectly level. MEASURE
    // THIS: at 19in up, a level camera cannot even see a floor ball closer
    // than ~2.5ft, so the camera is almost certainly angled down - and
    // X/Y/Z are only world-accurate once this matches reality.
    private static final double CAMERA_TILT_DEGREES = 0.0;
    // Optional sanity filter: a ball ON THE FLOOR must sit at
    // (ball radius - camera height) ~= 1.4 - 19 = -17.6in relative to the
    // lens (using the 2.8in Pollen diameter). A
    // detection whose height disagrees badly has a bogus size estimate
    // (phantom circle, non-ball blob) and can be rejected. ONLY enable this
    // after CAMERA_TILT_DEGREES is set accurately, or it will throw away
    // real balls. Also a good tool for rejecting furniture/wall detections,
    // since those almost never sit at "on the floor" height.
    private static final boolean HEIGHT_FILTER_ENABLED = false;
    private static final double HEIGHT_TOLERANCE_INCHES = 6.0;

    // ---- Tunable HSV thresholds (OpenCV HSV: H 0-179, S 0-255, V 0-255) ----
    // Pollen (yellow). Starting point for saturated yellow plastic under
    // typical indoor lighting - NOT an official spec, see class javadoc.
    // Raised the saturation/value floors vs. the old placeholder: dull
    // yellow-ish furniture/wood tends to sit lower on both S and V than a
    // glossy game ball, so this alone should cut a lot of false positives.
    // TUNE THIS with FTC Dashboard/Panels against your real balls.
    public static Scalar yellowLower = new Scalar(22, 140, 120);
    public static Scalar yellowUpper = new Scalar(32, 255, 255);

    private static final double MIN_CONTOUR_AREA = 400; // px^2, coarse pre-filter (raised from 200)
    private static final double MIN_RADIUS_PX = 10;      // minimum fitted-circle radius (raised from 8)

    // A lone, unobstructed ball scores near 1.0 on both of these. Anything
    // scoring low on either (peanut shape, crescent, non-ball blob) is
    // rejected rather than split apart.
    private static final double SINGLE_BALL_MIN_CIRCULARITY = 0.75;
    // Contour area / enclosing-circle area. A full disk is ~1.0.
    private static final double SINGLE_BALL_MIN_FILL = 0.75;

    // Fallback: a blob that fails the strict fast-path check above but is
    // still reasonably round is accepted as one ball (keeps a shadowed/
    // partially-lit ball from being dropped entirely).
    private static final double MIN_CIRCULARITY = 0.55; // raised from 0.45 - fewer furniture false positives

    // Working mats (allocated once, reused every frame - avoids GC churn)
    private final Mat blurred = new Mat();
    private final Mat hsvMat = new Mat();
    private final Mat yellowMask = new Mat();
    private final Mat morphKernel = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(5, 5));

    // ---- Debug/diagnostic state, exposed for telemetry ----
    private volatile long frameCount = 0;
    private volatile long lastProcessTimeNs = 0;
    private volatile String lastError = null;
    private final List<Detection> detections = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        long start = System.nanoTime();
        try {
            // Blur before thresholding - reduces mask noise/speckling that
            // otherwise fragments a single ball into multiple small blobs.
            Imgproc.GaussianBlur(input, blurred, new Size(7, 7), 0);
            Imgproc.cvtColor(blurred, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, yellowLower, yellowUpper, yellowMask);

            // OPEN (erode->dilate) clears small noise specks.
            // CLOSE (dilate->erode) fills small gaps/holes (glare, shadow)
            // so one ball stays one blob.
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, morphKernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, morphKernel);

            List<Detection> found = findBalls(yellowMask, ArtifactColor.YELLOW);

            // Swap in the new frame's results atomically so consumers never
            // see a half-built list.
            synchronized (this) {
                detections.clear();
                detections.addAll(found);
            }

            for (Detection d : found) {
                drawDetection(input, d, new Scalar(0, 255, 255));
            }

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

    private List<Detection> findBalls(Mat mask, ArtifactColor color) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();

        // Fill every blob solid. The pickleball-style balls have holes in
        // their surface, which punch dark spots into the yellow mask. Left
        // as-is, those spots create spurious interior edges/contours.
        // Painting the external contours filled erases the holes while
        // leaving the blob outlines - the only edges we actually care
        // about - intact.
        Imgproc.drawContours(mask, contours, -1, new Scalar(255), -1);

        List<Detection> found = new ArrayList<>();

        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area < MIN_CONTOUR_AREA) continue;

            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());
            double perimeter = Imgproc.arcLength(contour2f, true);
            Point circleCenter = new Point();
            float[] circleRadius = new float[1];
            Imgproc.minEnclosingCircle(contour2f, circleCenter, circleRadius);
            contour2f.release();

            if (perimeter <= 0 || circleRadius[0] < MIN_RADIUS_PX) continue;

            double circularity = (4 * Math.PI * area) / (perimeter * perimeter);
            double fillRatio = area / (Math.PI * circleRadius[0] * circleRadius[0]);

            // Round and solid -> accept as a single ball.
            if (circularity >= SINGLE_BALL_MIN_CIRCULARITY && fillRatio >= SINGLE_BALL_MIN_FILL) {
                found.add(makeDetection(color, Imgproc.boundingRect(c),
                        circleCenter, circleRadius[0], area, circularity));
                continue;
            }

            // Lenient fallback: still round enough to trust (e.g. partially
            // shadowed ball) even though it missed the strict fast path.
            if (circularity >= MIN_CIRCULARITY) {
                found.add(makeDetection(color, Imgproc.boundingRect(c),
                        circleCenter, circleRadius[0], area, circularity));
            }
            // Otherwise: irregular blob (merged balls, furniture edge, etc.)
            // is dropped rather than split apart.
        }

        if (HEIGHT_FILTER_ENABLED) {
            // A floor ball's center must be (ball radius - camera height)
            // below the lens. A detection that disagrees has a bogus size
            // estimate and isn't a ball sitting on the floor.
            double expectedY = BALL_DIAMETER_INCHES / 2.0 - CAMERA_HEIGHT_INCHES;
            found.removeIf(d -> Math.abs(d.y - expectedY) > HEIGHT_TOLERANCE_INCHES);
        }

        // Publish left-to-right: Ball #1 is always the leftmost ball in the
        // camera view, increasing toward the right. Keeps telemetry
        // numbering stable instead of following arbitrary contour order.
        found.sort((a, b) -> Double.compare(a.x, b.x));
        return found;
    }

    private Detection makeDetection(ArtifactColor color, Rect box, Point center,
                                    double radiusPx, double area, double score) {
        Detection d = new Detection();
        d.color = color;
        d.boundingBox = box;
        d.center = center;
        d.area = area;
        d.circularity = score;
        d.radiusPx = radiusPx;

        // Distance along the camera's optical axis, using known ball size vs
        // the fitted circle diameter.
        double pixelDiameter = radiusPx * 2.0;
        double zCam = (BALL_DIAMETER_INCHES * FOCAL_LENGTH_PIXELS) / pixelDiameter;

        // Offsets from the optical axis, via angle-off-centerline * distance.
        double pixelsPerDegreeH = FRAME_WIDTH / HORIZONTAL_FOV_DEGREES;
        double angleXDegrees = (center.x - FRAME_WIDTH / 2.0) / pixelsPerDegreeH;
        double xCam = zCam * Math.tan(Math.toRadians(angleXDegrees));

        double verticalFovDegrees = HORIZONTAL_FOV_DEGREES * FRAME_HEIGHT / FRAME_WIDTH;
        double pixelsPerDegreeV = FRAME_HEIGHT / verticalFovDegrees;
        double angleYDegrees = (center.y - FRAME_HEIGHT / 2.0) / pixelsPerDegreeV;
        // Image y grows DOWNWARD, so flip to make "up" positive.
        double yUpCam = -zCam * Math.tan(Math.toRadians(angleYDegrees));

        // Rotate the camera's downward pitch out of the reading so x/y/z
        // are level, real-world axes no matter how the camera is aimed:
        // z = horizontal distance along the floor, y = true height relative
        // to the lens (+ up). With CAMERA_TILT_DEGREES = 0 this is a no-op.
        double tilt = Math.toRadians(CAMERA_TILT_DEGREES);
        d.x = xCam;
        d.z = zCam * Math.cos(tilt) + yUpCam * Math.sin(tilt);
        d.y = -zCam * Math.sin(tilt) + yUpCam * Math.cos(tilt);
        return d;
    }

    private void drawDetection(Mat input, Detection d, Scalar drawColor) {
        Imgproc.circle(input, d.center, (int) Math.round(d.radiusPx), drawColor, 3);
        Imgproc.circle(input, d.center, 4, drawColor, -1);
        Imgproc.putText(input,
                String.format("Z=%.1f\" s=%.2f", d.z, d.circularity),
                new Point(d.boundingBox.x, Math.max(d.boundingBox.y - 8, 12)),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, drawColor, 2);
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
        yellowLower = lower;
        yellowUpper = upper;
    }
}