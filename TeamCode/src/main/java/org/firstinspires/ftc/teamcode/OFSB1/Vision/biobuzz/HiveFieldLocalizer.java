package org.firstinspires.ftc.teamcode.OFSB1.Vision.biobuzz;

import org.firstinspires.ftc.easyatl.EasyATL;
import org.firstinspires.ftc.easyatl.FieldPose;
import org.firstinspires.ftc.easyatl.FtcEasyATL;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Robot position on the field from the HIVE tags, using EasyATL's public API.
 * Reads detections the HIVE camera already produced. Does not open a camera,
 * write into Pedro, or change which cell is shootable.
 */
public final class HiveFieldLocalizer {

    private final FtcEasyATL localizer;
    private FieldPose pose;
    private boolean accepted;
    private String error;

    public HiveFieldLocalizer() {
        localizer = new FtcEasyATL(camera(), pipeline(), HiveFieldMap.tags());
    }

    public void update(List<AprilTagDetection> detections) {
        try {
            accepted = localizer.localize(detections);
            if (localizer.hasPose()) {
                pose = localizer.getPose();
            }
            error = null;
        } catch (RuntimeException exception) {
            accepted = false;
            error = exception.getClass().getSimpleName();
        }
    }

    public boolean hasPose() {
        return pose != null;
    }

    public FieldPose getPose() {
        return pose;
    }

    public boolean acceptedThisFrame() {
        return accepted;
    }

    public double getQuality() {
        try {
            return localizer.getQuality();
        } catch (RuntimeException exception) {
            return 0;
        }
    }

    public String getError() {
        return error;
    }

    /**
     * Field coordinates of a point that is {@code rightInches} to the robot's
     * right and {@code forwardInches} in front of the robot center.
     * Returns null until EasyATL has a pose. Heading 0 is field +X.
     */
    public double[] toField(double rightInches, double forwardInches) {
        if (pose == null || !Double.isFinite(rightInches) || !Double.isFinite(forwardInches)) {
            return null;
        }
        double heading = pose.heading;
        double x = pose.x + forwardInches * Math.cos(heading) + rightInches * Math.sin(heading);
        double y = pose.y + forwardInches * Math.sin(heading) - rightInches * Math.cos(heading);
        return new double[] { x, y };
    }

    private static EasyATL.CameraConfig camera() {
        return new EasyATL.CameraConfig(
                BioBuzzVisionConfig.CAMERA_FORWARD_OF_CENTER_INCHES,
                BioBuzzVisionConfig.CAMERA_RIGHT_OF_CENTER_INCHES,
                BioBuzzVisionConfig.CAMERA_YAW_RADIANS,
                BioBuzzVisionConfig.CAMERA_PITCH_RADIANS);
    }

    private static EasyATL.Config pipeline() {
        return new EasyATL.Config()
                .setMaxRangeInches(BioBuzzVisionConfig.FIELD_MAX_RANGE_INCHES)
                .setMaxBearingDegrees(BioBuzzVisionConfig.FIELD_MAX_BEARING_DEGREES)
                .setMaxTagYawDegrees(BioBuzzVisionConfig.FIELD_MAX_TAG_YAW_DEGREES)
                .setOutlierDistanceInches(BioBuzzVisionConfig.FIELD_OUTLIER_DISTANCE_INCHES)
                .setOutlierHeadingDegrees(BioBuzzVisionConfig.FIELD_OUTLIER_HEADING_DEGREES)
                .setSmoothingAlpha(BioBuzzVisionConfig.FIELD_SMOOTHING_ALPHA)
                .setMaxStepInches(BioBuzzVisionConfig.FIELD_MAX_STEP_INCHES)
                .setMaxStepDegrees(BioBuzzVisionConfig.FIELD_MAX_STEP_DEGREES);
    }
}
