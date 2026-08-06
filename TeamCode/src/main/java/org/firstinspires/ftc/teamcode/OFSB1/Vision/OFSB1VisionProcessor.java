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
 * Detects BIOBUZZ-season POLLEN (yellow ~2.8-3in balls) using HSV color
 * thresholding + watershed-based separation, so balls that are touching or
 * overlapping in the camera view are still reported as separate detections
 * instead of merging into one blob. Circularity check remains lenient so a
 * partially-shadowed or partially-occluded (including partly cut off at the
 * frame edge, or partly overlapped by a neighboring ball) ball still counts.
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

    private static final double MIN_CONTOUR_AREA = 400; // px^2, filters noise
    // Lenient on purpose - a ball with a shadow across part of it, partly cut
    // off at the frame edge, or partly overlapped by a neighboring ball,
    // should still pass. A full circle scores near 1.0; a ball split from a
    // touching neighbor by watershed still scores fairly high (a straight
    // cut near the tangent point removes little area). This only rejects
    // shapes that are clearly NOT round at all (tape lines, jerseys, panels).
    private static final double MIN_CIRCULARITY = 0.45;

    // ---- Watershed ball-separation tuning ----
    // Non-max-suppression radius (px) used to find one "peak" per ball in
    // the distance transform. Should be smaller than the smallest expected
    // ball radius in pixels (so each ball still gets its own peak) but big
    // enough to merge noisy multi-peaks within a single ball into one.
    // TUNE THIS alongside FOCAL_LENGTH_PIXELS - balls further away are
    // smaller in pixels and need a smaller radius here to still get a peak.
    private static final int PEAK_NMS_RADIUS_PX = 9;
    // Minimum distance-transform value (px from the nearest mask edge) for a
    // pixel to be eligible as a peak at all. Filters out shallow noise blobs
    // that survived the open() step but are too thin/small to be a ball.
    private static final double MIN_PEAK_DEPTH_PX = 6.0;
    // How far (px) to dilate the mask to build the "definitely background or
    // boundary" region for watershed. A few px is enough; this just defines
    // the fuzzy boundary band watershed is allowed to draw the split through.
    private static final int BG_DILATE_ITERATIONS = 3;

    // Working mats (allocated once, reused every frame - avoids GC churn)
    private final Mat hsvMat = new Mat();
    private final Mat yellowMask = new Mat();
    private final Mat distMat = new Mat();
    private final Mat dilatedDistMat = new Mat();
    private final Mat peakMask = new Mat();
    private final Mat depthMask = new Mat();
    private final Mat sureFgMat = new Mat();
    private final Mat sureBgMat = new Mat();
    private final Mat unknownMat = new Mat();
    private final Mat markersMat = new Mat();
    private final Mat markerInputMat = new Mat();
    private final Mat labelMaskMat = new Mat();
    private final Mat peakKernel =
            Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,
                    new Size(2 * PEAK_NMS_RADIUS_PX + 1, 2 * PEAK_NMS_RADIUS_PX + 1));
    private final Mat bgKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 3));

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

            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, yellowLower, yellowUpper, yellowMask);

            // Erode+dilate (open) to knock out small noise specks like tape
            // lines or reflections before separating/detecting balls.
            Imgproc.erode(yellowMask, yellowMask, new Mat());
            Imgproc.dilate(yellowMask, yellowMask, new Mat());

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

    /**
     * Separates the binary color mask into individual ball regions (even if
     * touching/overlapping) via distance-transform peak seeding + watershed,
     * then measures and reports each region exactly like a normal contour.
     */
    private void findAndDraw(Mat input, Mat mask, ArtifactColor color, Scalar drawColor) {
        // 1) Distance transform: every foreground pixel's value = distance
        //    (px) to the nearest background pixel. Each ball's center forms
        //    a local peak here, even when two balls are touching, because
        //    the peaks are pulled toward each ball's own center of mass.
        Imgproc.distanceTransform(mask, distMat, Imgproc.DIST_L2, 5);

        // 2) Find local maxima (one seed per ball) via non-max suppression:
        //    a pixel is a peak if it equals the max of its neighborhood.
        Imgproc.dilate(distMat, dilatedDistMat, peakKernel);
        Core.compare(distMat, dilatedDistMat, peakMask, Core.CMP_EQ);
        // Reject shallow "peaks" from thin noise shapes, not just true balls.
        Imgproc.threshold(distMat, depthMask, MIN_PEAK_DEPTH_PX, 255, Imgproc.THRESH_BINARY);
        depthMask.convertTo(depthMask, CvType.CV_8U);
        Core.bitwise_and(peakMask, depthMask, sureFgMat);

        // 3) Build the "definitely background or boundary" region: dilate
        //    the mask outward a few px. The gap between this and the seeds
        //    (sureFgMat) is the ambiguous band watershed is allowed to cut
        //    the split line through.
        Imgproc.dilate(mask, sureBgMat, bgKernel, new Point(-1, -1), BG_DILATE_ITERATIONS);
        Core.subtract(sureBgMat, sureFgMat, unknownMat);

        // 4) Label each seed as its own marker, shift labels so background
        //    (non-seed, label 0 from connectedComponents) becomes 1, and
        //    mark the ambiguous band as 0 (unknown) for watershed to fill in.
        int numSeeds = Imgproc.connectedComponents(sureFgMat, markersMat, 8, CvType.CV_32S);
        if (numSeeds <= 1) {
            // No balls in frame at all - nothing to split, nothing to report.
            return;
        }
        Core.add(markersMat, new Scalar(1), markersMat);
        markersMat.setTo(new Scalar(0), unknownMat);

        // watershed() requires a 3-channel 8-bit image to run on; it doesn't
        // use the color data meaningfully here (mask already isolated the
        // balls), it just needs *some* valid 3-channel image of that size.
        if (input.channels() == 4) {
            Imgproc.cvtColor(input, markerInputMat, Imgproc.COLOR_RGBA2RGB);
        } else {
            input.copyTo(markerInputMat);
        }
        Imgproc.watershed(markerInputMat, markersMat);

        // 5) Each label from 2..numSeeds+1 is now one separated ball region.
        //    (Label 1 = background, -1 = boundary lines watershed drew.)
        for (int label = 2; label <= numSeeds; label++) {
            Core.inRange(markersMat, new Scalar(label), new Scalar(label), labelMaskMat);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(labelMaskMat, contours, hierarchy,
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            hierarchy.release();
            if (contours.isEmpty()) continue;

            // A single watershed region should yield one contour; if noise
            // produced more than one, just take the largest.
            MatOfPoint c = contours.get(0);
            double bestArea = Imgproc.contourArea(c);
            for (MatOfPoint candidate : contours) {
                double a = Imgproc.contourArea(candidate);
                if (a > bestArea) {
                    bestArea = a;
                    c = candidate;
                }
            }

            double area = bestArea;
            if (area < MIN_CONTOUR_AREA) continue;

            double perimeter = Imgproc.arcLength(new MatOfPoint2f(c.toArray()), true);
            if (perimeter <= 0) continue;
            double circularity = (4 * Math.PI * area) / (perimeter * perimeter);
            if (circularity < MIN_CIRCULARITY) continue; // not round enough - skip (e.g. tape, jersey)

            Rect box = Imgproc.boundingRect(c);
            Detection d = new Detection();
            d.color = color;
            d.boundingBox = box;
            d.center = new Point(box.x + box.width / 2.0, box.y + box.height / 2.0);
            d.area = area;
            d.circularity = circularity;

            // Z: distance from camera to ball, using known ball size vs apparent pixel width
            d.z = (BALL_DIAMETER_INCHES * FOCAL_LENGTH_PIXELS) / box.width;

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

            Imgproc.rectangle(input, box, drawColor, 3);
            Imgproc.circle(input, d.center, 5, drawColor, -1);
            Imgproc.putText(input,
                    String.format("Z=%.1f\" c=%.2f", d.z, d.circularity),
                    new Point(box.x, Math.max(box.y - 8, 12)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, drawColor, 2);
        }
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