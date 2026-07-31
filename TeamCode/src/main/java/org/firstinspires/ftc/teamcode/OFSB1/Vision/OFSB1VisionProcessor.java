package org.firstinspires.ftc.teamcode.OFSB1.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Detects BIOBUZZ-season POLLEN (yellow ~2.8-3in balls) using HSV color
 * thresholding + contour analysis.
 *
 * IMPORTANT: The HSV ranges below are starting points. Real lighting in your
 * gym/field WILL differ from these. Tune them with FTC Dashboard (see notes
 * at bottom) rather than guessing blind.
 */
public class OFSB1VisionProcessor extends OpenCvPipeline {

    public enum ArtifactColor { YELLOW }

    public static class Detection {
        public ArtifactColor color;
        public Rect boundingBox;
        public Point center;
        public double area;
    }

    // ---- Tunable HSV thresholds (OpenCV HSV: H 0-179, S 0-255, V 0-255) ----
    // Pollen (yellow, BIOBUZZ 2026-2027) - starting guess, TUNE THIS.
    // Yellow sits near skin-tone/wood-floor hues, so keep saturation/value
    // floors reasonably high to reject washed-out background colors.
    private Scalar yellowLower = new Scalar(20, 100, 100);
    private Scalar yellowUpper = new Scalar(35, 255, 255);

    private static final double MIN_CONTOUR_AREA = 400; // px^2, filters noise

    // Working mats (allocated once, reused every frame - avoids GC churn)
    private final Mat hsvMat = new Mat();
    private final Mat yellowMask = new Mat();

    // ---- Debug/diagnostic state, exposed for telemetry ----
    private volatile long frameCount = 0;
    private volatile long lastProcessTimeNs = 0;
    private volatile String lastError = null;
    private final List<Detection> detections = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        long start = System.nanoTime();
        try {
            detections.clear();

            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, yellowLower, yellowUpper, yellowMask);

            // Erode+dilate (open) to knock out small noise specks like tape
            // lines or reflections before contour detection.
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

    private void findAndDraw(Mat input, Mat mask, ArtifactColor color, Scalar drawColor) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area < MIN_CONTOUR_AREA) continue;

            Rect box = Imgproc.boundingRect(c);
            Detection d = new Detection();
            d.color = color;
            d.boundingBox = box;
            d.center = new Point(box.x + box.width / 2.0, box.y + box.height / 2.0);
            d.area = area;
            detections.add(d);

            Imgproc.rectangle(input, box, drawColor, 3);
            Imgproc.circle(input, d.center, 5, drawColor, -1);
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

