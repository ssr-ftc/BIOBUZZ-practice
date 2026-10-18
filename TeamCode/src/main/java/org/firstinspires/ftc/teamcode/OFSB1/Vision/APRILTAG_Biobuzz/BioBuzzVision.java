package org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.easyatl.FieldPose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * TeleOp-facing HIVE vision. Call {@link #update(double)} once per loop,
 * then read {@link #getState()}. This class does not drive or shoot.
 *
 * <p>Safe when the camera sees nothing: the state is empty, shootable is
 * false, and no heading command is produced here.
 */
public final class BioBuzzVision {

    private final AprilTagDetector detector = new AprilTagDetector();
    private final TargetSelector selector = new TargetSelector();
    private final HiveFieldLocalizer field = new HiveFieldLocalizer();

    private VisionState state = VisionState.empty(TargetStatus.SEARCHING, TargetBlockReason.NO_RELEVANT_TAGS);
    private List<AprilTagDetection> rawDetections = Collections.emptyList();
    private TagClusterMapper.FilterResult lastFilter = new TagClusterMapper.FilterResult(
            Collections.<HiveCluster, List<AprilTagDetection>>emptyMap(), 0, 0, 0);
    private List<HiveCell> visibleCells = Collections.emptyList();

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        selector.reset();
        detector.init(hardwareMap, telemetry);
    }

    public void update(double nowSeconds) {
        detector.update();
        rawDetections = detector.getDetections();
        field.update(rawDetections);

        Alliance alliance = AllianceConfig.get();
        lastFilter = TagClusterMapper.filter(rawDetections, alliance);
        visibleCells = HivePoseEstimator.estimate(lastFilter);
        state = selector.select(
                visibleCells,
                lastFilter,
                alliance,
                detector.isStreaming(),
                nowSeconds);
    }

    public VisionState getState() {
        return state;
    }

    public boolean hasTarget() {
        return state.hasTarget();
    }

    public boolean hasValidTarget() {
        return state.hasValidTarget();
    }

    public ShootingTarget getTarget() {
        return state.getTarget();
    }

    public ShootingTarget getRecommendedTarget() {
        return state.getRecommendedTarget();
    }

    public double getTargetDistance() {
        return state.getTargetDistance();
    }

    public double getTargetBearing() {
        return state.getTargetBearing();
    }

    public boolean isTargetShootable() {
        return state.isTargetShootable();
    }

    public TargetConfidence getTargetConfidence() {
        return state.getTargetConfidence();
    }

    public void stop() {
        detector.stop();
    }

    public void addTelemetry(Telemetry telemetry, boolean debug) {
        telemetry.addLine("BIOBUZZ VISION");
        Alliance alliance = AllianceConfig.get();
        telemetry.addData("Alliance", alliance == null ? "NOT SET" : alliance.name());

        ShootingTarget target = state.getTarget();
        HiveCell cell = target.cell;

        if (alliance == null) {
            telemetry.addData("Shootable", "NO");
            telemetry.addData("Reason", TargetBlockReason.ALLIANCE_NOT_SELECTED.telemetry);
        } else {
            telemetry.addLine("Same-alliance cells");
            boolean anyShootable = false;
            for (HiveCluster cluster : HiveCluster.values()) {
                if (cluster.alliance != alliance) {
                    continue;
                }
                HiveCell visible = findCell(cluster);
                if (visible == null) {
                    telemetry.addData(cluster.displayName(), "NOT VISIBLE | SHOOTABLE NO");
                    continue;
                }
                boolean shootable = TargetSelector.isShootable(visible);
                anyShootable = anyShootable || shootable;
                telemetry.addData(cluster.displayName(), "%s | SHOOTABLE %s | tags %s",
                        orientationLabel(visible.orientation),
                        shootable ? "YES" : "NO",
                        formatIds(visible.visibleTagIds));
            }
            telemetry.addData("Shootable", anyShootable ? "YES" : "NO");
        }

        addFieldTelemetry(telemetry, cell);

        if (state.hasTarget() && cell != null) {
            telemetry.addData("Distance", formatInches(cell.distance));
            telemetry.addData("Bearing", formatDegrees(cell.bearing));
            telemetry.addData("Camera X", formatInches(cell.relativeX));
            telemetry.addData("Camera Y", formatInches(cell.relativeY));
            telemetry.addData("Target", statusLabel(state.getStatus()));
            telemetry.addData("Confidence", state.getTargetConfidence().name());
            if (state.getReason() != TargetBlockReason.NONE) {
                telemetry.addData("Reason", state.getReason().telemetry);
            }
        } else if (alliance != null) {
            telemetry.addData("Target", "NONE");
            if (state.getReason() != TargetBlockReason.NONE) {
                telemetry.addData("Reason", state.getReason().telemetry);
            }
        }

        if (!debug) {
            return;
        }

        telemetry.addLine("---- debug ----");
        telemetry.addData("Camera", detector.isStreaming() ? "STREAMING" : "NOT STREAMING");
        telemetry.addData("Status", state.getStatus().name());
        telemetry.addData("Unknown tags", lastFilter.unknownTagCount);
        telemetry.addData("Opposing tags", lastFilter.opposingTagCount);
        telemetry.addData("Rejected hamming", lastFilter.rejectedHammingCount);
        if (field.getError() != null) {
            telemetry.addData("Field error", field.getError());
        }
        if (cell != null && cell.poseValid) {
            telemetry.addData("Pose XYZ", "%.1f  %.1f  %.1f in", cell.relativeX, cell.relativeY, cell.relativeZ);
            telemetry.addData("Pose yaw pitch roll", "%.1f  %.1f  %.1f deg", cell.yaw, cell.pitch, cell.roll);
            telemetry.addData("3D range", "%.1f in", cell.range);
            telemetry.addData("Decision margin", "%.1f", cell.meanDecisionMargin);
            telemetry.addData("Pose tags", cell.poseTagCount);
        }
        telemetry.addLine("All tags before filtering");
        if (rawDetections.isEmpty()) {
            telemetry.addLine("(none)");
        }
        for (AprilTagDetection detection : rawDetections) {
            HiveCluster cluster = TagClusterMapper.getCellForTagId(detection.id);
            String clusterName = cluster == null ? "IGNORED" : cluster.displayName();
            if (detection.ftcPose != null && detection.metadata != null) {
                telemetry.addLine(String.format(
                        Locale.US,
                        "id %d  %s  ham %d  margin %.0f  range %.1f  bearing %.1f  roll %.1f",
                        detection.id,
                        clusterName,
                        detection.hamming,
                        detection.decisionMargin,
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.roll));
            } else {
                telemetry.addLine(String.format(
                        Locale.US,
                        "id %d  %s  ham %d  margin %.0f  NO POSE",
                        detection.id,
                        clusterName,
                        detection.hamming,
                        detection.decisionMargin));
            }
        }
    }

    private void addFieldTelemetry(Telemetry telemetry, HiveCell cell) {
        telemetry.addLine("FIELD");
        if (!field.hasPose()) {
            telemetry.addData("Field pose", "NONE");
            return;
        }
        FieldPose pose = field.getPose();
        telemetry.addData("Field X", "%.1f in", pose.x);
        telemetry.addData("Field Y", "%.1f in", pose.y);
        telemetry.addData("Field heading", "%.1f deg", pose.headingDegrees());
        telemetry.addData("Field quality", "%.0f%%", 100.0 * field.getQuality());
        if (cell != null) {
            double[] target = field.toField(cell.relativeX, cell.relativeY);
            if (target != null) {
                telemetry.addData("Target field X", "%.1f in", target[0]);
                telemetry.addData("Target field Y", "%.1f in", target[1]);
            }
        }
    }

    private static String statusLabel(TargetStatus status) {
        switch (status) {
            case TARGET_VALID:
                return "VALID";
            case TARGET_INVALID:
                return "INVALID";
            case LOW_CONFIDENCE:
                return "LOW CONFIDENCE";
            case TEMPORARILY_LOST:
                return "LOST (HOLDING)";
            case SEARCHING:
            case NO_TARGET:
            default:
                return "NONE";
        }
    }

    private static String orientationLabel(CellOrientation orientation) {
        switch (orientation) {
            case FACING_UP:
                return "FACING UP";
            case FACING_DOWN:
                return "FACING DOWN";
            case UNKNOWN:
            default:
                return "UNKNOWN";
        }
    }

    private static String formatIds(int[] ids) {
        if (ids == null || ids.length == 0) {
            return "(none)";
        }
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < ids.length; i++) {
            if (i > 0) {
                builder.append(", ");
            }
            builder.append(ids[i]);
        }
        return builder.toString();
    }

    private static String formatInches(double inches) {
        if (!Double.isFinite(inches)) {
            return "UNKNOWN";
        }
        return String.format(Locale.US, "%.1f in", inches);
    }

    private static String formatDegrees(double degrees) {
        if (!Double.isFinite(degrees)) {
            return "UNKNOWN";
        }
        return String.format(Locale.US, "%.1f deg", degrees);
    }

    /** Visible alliance cells from the latest update. Useful for autonomous later. */
    public List<HiveCell> getVisibleCells() {
        return new ArrayList<>(visibleCells);
    }

    private HiveCell findCell(HiveCluster cluster) {
        for (HiveCell cell : visibleCells) {
            if (cell.cluster == cluster) {
                return cell;
            }
        }
        return null;
    }
}
