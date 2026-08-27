package org.firstinspires.ftc.teamcode.OFSB1.Vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
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

/**
 * Detects POLLEN balls (yellow ~5in enlarged-pickleball-style balls with
 * holes in the surface) using HSV color thresholding + contour analysis,
 * plus a Hough-circle "splitter" that separates overlapping/occluded balls
 * which merge into one yellow blob. Blob interiors are filled solid before
 * analysis so the surface holes can't register as balls of their own.
 *
 * Detections are published sorted left-to-right (Ball #1 = leftmost).
 *
 * Two-tier design (accuracy without per-frame lag):
 *   1. FAST PATH (every blob): if a yellow blob is round AND solid, it is
 *      almost certainly a single ball - accept it with cheap contour math.
 *   2. SPLIT PATH (suspicious blobs only): if the blob is NOT round/solid
 *      (peanut shape = two merged balls, crescent = ball peeking out from
 *      behind another), run HoughCircles on just that blob's small region
 *      of the mask. Hough votes on edge ARCS, so it recovers each ball's
 *      true circle even when only part of its outline is visible.
 *
 * The expensive step runs only where needed, only on small sub-images, and
 * is hard-capped per frame - so a clean scene costs the same as the old
 * contour-only pipeline.
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
        // Confidence-ish score in 0..1:
        //  - fast-path detections: classic circularity (4*pi*A / P^2)
        //  - Hough-split detections: fraction of the fitted circle that is
        //    actually yellow mask (lower for occluded balls, by design)
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
    // Real-world ball diameter (inches). Our practice balls are ~5in
    // enlarged-pickleball-style balls (with holes in the surface).
    private static final double BALL_DIAMETER_INCHES = 5.0;

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
    // (ball radius - camera height) ~= -16.5in relative to the lens. A
    // detection whose height disagrees badly has a bogus size estimate
    // (phantom circle, merged blob) and can be rejected. ONLY enable this
    // after CAMERA_TILT_DEGREES is set accurately, or it will throw away
    // real balls.
    private static final boolean HEIGHT_FILTER_ENABLED = false;
    private static final double HEIGHT_TOLERANCE_INCHES = 6.0;

    // ---- Tunable HSV thresholds (OpenCV HSV: H 0-179, S 0-255, V 0-255) ----
    // Pollen (yellow) - starting guess, TUNE THIS.
    private Scalar yellowLower = new Scalar(20, 100, 100);
    private Scalar yellowUpper = new Scalar(35, 255, 255);

    private static final double MIN_CONTOUR_AREA = 200; // px^2, coarse pre-filter
    private static final double MIN_RADIUS_PX = 8;      // minimum fitted-circle radius

    // ---- Fast path vs split path decision ----
    // A lone, unobstructed ball scores near 1.0 on BOTH of these. A merged
    // blob (peanut) or occluded ball (crescent) scores low on at least one,
    // which routes it to the Hough splitter instead.
    private static final double SINGLE_BALL_MIN_CIRCULARITY = 0.75;
    // Contour area / enclosing-circle area. A full disk is ~1.0; a peanut
    // inside its enclosing circle is ~0.5-0.65.
    private static final double SINGLE_BALL_MIN_FILL = 0.75;

    // Fallback: if the splitter finds nothing, a blob at least this round is
    // still accepted as one ball (matches the old pipeline's leniency).
    private static final double MIN_CIRCULARITY = 0.45;

    // ---- Hough splitter tuning ----
    // Hard cap on splitter runs per frame so a pathological scene (yellow
    // banner, bad HSV tuning) can't stall the pipeline.
    private static final int MAX_HOUGH_BLOBS_PER_FRAME = 4;
    private static final int MAX_CIRCLES_PER_BLOB = 4;
    // Canny high threshold used internally by HoughCircles.
    private static final double HOUGH_CANNY_THRESHOLD = 120;
    // Accumulator votes needed to accept a circle. LOWER finds fainter /
    // more-occluded arcs but risks phantom circles; raise if you see
    // circles appearing on non-balls.
    private static final double HOUGH_VOTES_THRESHOLD = 20;
    // A fitted circle must have at least this fraction of its area covered
    // by yellow mask to count. A fully visible ball is ~0.9+; a half-hidden
    // ball can be ~0.3, hence the low bar.
    private static final double MIN_MASK_COVERAGE = 0.28;

    // Working mats (allocated once, reused every frame - avoids GC churn)
    private final Mat blurred = new Mat();
    private final Mat hsvMat = new Mat();
    private final Mat yellowMask = new Mat();
    private final Mat houghInput = new Mat();
    private final Mat houghCircles = new Mat();
    private final Mat morphKernel = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(5, 5));

    // ---- Debug/diagnostic state, exposed for telemetry ----
    private volatile long frameCount = 0;
    private volatile long lastProcessTimeNs = 0;
    private volatile int houghRunsLastFrame = 0;
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
            // so one ball stays one blob. Note CLOSE can also glue two
            // near-touching balls together - that's fine now, because the
            // Hough splitter pulls merged blobs back apart.
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
        // as-is, those spots create interior edges that the Hough splitter
        // happily fits small circles to ("ball inside a ball") and they
        // also drag down the mask-coverage score of real balls. Painting
        // the external contours filled erases the holes while leaving the
        // blob outlines - the only edges we actually care about - intact.
        Imgproc.drawContours(mask, contours, -1, new Scalar(255), -1);

        List<Detection> found = new ArrayList<>();
        int houghBudget = MAX_HOUGH_BLOBS_PER_FRAME;
        int houghRuns = 0;

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

            // FAST PATH: round and solid -> a single unobstructed ball.
            if (circularity >= SINGLE_BALL_MIN_CIRCULARITY && fillRatio >= SINGLE_BALL_MIN_FILL) {
                found.add(makeDetection(color, Imgproc.boundingRect(c),
                        circleCenter, circleRadius[0], area, circularity));
                continue;
            }

            // SPLIT PATH: peanut/crescent-shaped blob - probably 2+ merged
            // balls, or one ball partly hidden behind another.
            boolean split = false;
            if (houghBudget > 0) {
                houghBudget--;
                houghRuns++;
                split = splitBlobWithHough(mask, c, circleRadius[0], color, found);
            }

            // Splitter found nothing (or budget exhausted): keep the old
            // lenient behavior so a shadowed single ball isn't dropped.
            if (!split && circularity >= MIN_CIRCULARITY) {
                found.add(makeDetection(color, Imgproc.boundingRect(c),
                        circleCenter, circleRadius[0], area, circularity));
            }
        }

        houghRunsLastFrame = houghRuns;

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

    /**
     * Runs HoughCircles on just this blob's padded bounding box within the
     * mask. Returns true if at least one validated circle was added.
     */
    private boolean splitBlobWithHough(Mat mask, MatOfPoint contour, float blobRadius,
                                       ArtifactColor color, List<Detection> out) {
        Rect box = Imgproc.boundingRect(contour);
        int pad = 8;
        int x = Math.max(box.x - pad, 0);
        int y = Math.max(box.y - pad, 0);
        int w = Math.min(box.width + 2 * pad, mask.cols() - x);
        int h = Math.min(box.height + 2 * pad, mask.rows() - y);
        Mat roi = mask.submat(new Rect(x, y, w, h));

        // Slight blur gives Hough's internal edge detector a smooth gradient
        // instead of a hard binary step.
        Imgproc.GaussianBlur(roi, houghInput, new Size(5, 5), 0);
        roi.release();

        // Each ball in a merged pair is roughly half the blob's enclosing
        // radius, so centers should be at least ~half a blob-radius apart.
        double minDist = Math.max(MIN_RADIUS_PX, blobRadius * 0.5);
        // A real ball is a big fraction of its blob (half of a merged pair,
        // ~all of a lone blob). The surface holes of the pickleball-style
        // balls are FAR smaller than that, so a per-blob minimum radius
        // rejects hole-sized circles even if their edges survive the
        // hole-filling above.
        int minR = (int) Math.max(MIN_RADIUS_PX, blobRadius * 0.3);
        int maxR = (int) Math.ceil(blobRadius) + 2;

        Imgproc.HoughCircles(houghInput, houghCircles, Imgproc.HOUGH_GRADIENT,
                2.0,   // dp=2: accumulator at half resolution - faster, still accurate
                minDist,
                HOUGH_CANNY_THRESHOLD,
                HOUGH_VOTES_THRESHOLD,
                minR, maxR);

        int added = 0;
        for (int i = 0; i < houghCircles.cols() && added < MAX_CIRCLES_PER_BLOB; i++) {
            double[] circ = houghCircles.get(0, i);
            if (circ == null) continue;
            double cx = circ[0] + x;
            double cy = circ[1] + y;
            double r = circ[2];

            // Reject circles that aren't actually sitting on yellow pixels
            // (Hough can hallucinate arcs from mask noise).
            double coverage = maskCoverage(mask, cx, cy, r);
            if (coverage < MIN_MASK_COVERAGE) continue;

            int bx = (int) Math.max(cx - r, 0);
            int by = (int) Math.max(cy - r, 0);
            int bw = (int) Math.min(2 * r, mask.cols() - bx);
            int bh = (int) Math.min(2 * r, mask.rows() - by);
            Rect circleBox = new Rect(bx, by, bw, bh);

            // Area = yellow pixels inside the fitted circle; circularity
            // field carries the coverage score for these detections.
            out.add(makeDetection(color, circleBox, new Point(cx, cy), r,
                    coverage * Math.PI * r * r, coverage));
            added++;
        }
        return added > 0;
    }

    /** Fraction of the circle's area that is "on" in the mask (0..1). */
    private double maskCoverage(Mat mask, double cx, double cy, double r) {
        int x0 = (int) Math.floor(Math.max(cx - r, 0));
        int y0 = (int) Math.floor(Math.max(cy - r, 0));
        int x1 = (int) Math.ceil(Math.min(cx + r, mask.cols()));
        int y1 = (int) Math.ceil(Math.min(cy + r, mask.rows()));
        if (x1 - x0 <= 0 || y1 - y0 <= 0) return 0;

        Mat roi = mask.submat(new Rect(x0, y0, x1 - x0, y1 - y0));
        Mat circleMask = Mat.zeros(roi.size(), CvType.CV_8UC1);
        Imgproc.circle(circleMask, new Point(cx - x0, cy - y0),
                (int) Math.round(r), new Scalar(255), -1);
        Core.bitwise_and(roi, circleMask, circleMask);
        int onPixels = Core.countNonZero(circleMask);
        circleMask.release();
        roi.release();
        return onPixels / (Math.PI * r * r);
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
        // the FITTED circle diameter (correct even when the ball is partly
        // hidden, unlike the merged blob's size).
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

    /**
     * How many blobs needed the Hough splitter last frame (0 on a clean
     * scene). If this is pinned at MAX_HOUGH_BLOBS_PER_FRAME, your HSV range
     * is probably matching non-ball stuff.
     */
    public int getHoughRunsLastFrame() {
        return houghRunsLastFrame;
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
