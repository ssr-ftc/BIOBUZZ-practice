package org.firstinspires.ftc.teamcode.Mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class AprilTagWebcam {

    // ---- Detection reliability tuning ----
    // Short manual exposure kills motion blur (the #1 reason a tag you're
    // looking straight at flickers in and out while the robot moves), and
    // high gain compensates for the darker image. If the image on the
    // camera stream looks too dark to see the tag at all, raise EXPOSURE_MS
    // to 8-10; if detection still drops while moving, lower it to 3-4.
    private static final long EXPOSURE_MS = 6;
    private static final int GAIN = 250; // clamped to the camera's max below

    // Detector downsampling. LOWER = detects smaller/farther tags but costs
    // CPU. 3 is the SDK default; 2 roughly doubles usable detection range.
    private static final float DECIMATION = 2;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;
    private boolean cameraSettingsApplied = false;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                // INCH, not CM - the telemetry labels below and all robot
                // geometry math (camera offsets, stop distances) are inches.
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        aprilTagProcessor.setDecimation(DECIMATION);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        // MJPEG instead of uncompressed YUY2: the camera can deliver full
        // 30fps, so the detector gets ~3x more chances per second to see
        // the tag (YUY2 at 640x480 is often capped much lower over USB).
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
        cameraSettingsApplied = false;
    }

    public void update() {
        // Exposure/gain can only be set once the camera is actually
        // streaming, so apply them lazily on the first update() after
        // startup instead of blocking init().
        if (!cameraSettingsApplied
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            applyCameraSettings();
            cameraSettingsApplied = true;
        }
        detectedTags = aprilTagProcessor.getDetections();
    }

    /**
     * Locks exposure short and cranks gain. Auto-exposure picks long, blurry
     * exposures in gym lighting - motion blur then destroys the tag's sharp
     * black/white edges, which is exactly what the detector keys on.
     */
    private void applyCameraSettings() {
        try {
            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            if (exposure != null && exposure.isModeSupported(ExposureControl.Mode.Manual)) {
                exposure.setMode(ExposureControl.Mode.Manual);
                long min = exposure.getMinExposure(TimeUnit.MILLISECONDS);
                long max = exposure.getMaxExposure(TimeUnit.MILLISECONDS);
                exposure.setExposure(Math.max(min, Math.min(EXPOSURE_MS, max)), TimeUnit.MILLISECONDS);
            }

            GainControl gain = visionPortal.getCameraControl(GainControl.class);
            if (gain != null) {
                gain.setGain(Math.min(GAIN, gain.getMaxGain()));
            }
        } catch (Exception e) {
            // Some webcams don't expose these controls - detection still
            // works on auto settings, just less reliably while moving.
            telemetry.addData("Camera settings", "manual exposure unsupported: %s", e.getMessage());
        }
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) {return;}

        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }
    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id){
                return detection;
            }
        }
        return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
