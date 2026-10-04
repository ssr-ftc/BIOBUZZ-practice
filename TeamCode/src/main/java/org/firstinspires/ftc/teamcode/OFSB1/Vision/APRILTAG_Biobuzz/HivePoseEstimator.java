package org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Turns every tag that belongs to one CELL into one robot-relative pose.
 * Image pixel position is not used. The SDK pose is already in inches.
 */
public final class HivePoseEstimator {

    // Tags whose position disagrees with the cluster's median by more than
    // this are excluded before averaging - protects against one bad read
    // (glare, partial occlusion, a bad viewing angle on just that tag)
    // pulling the fused pose off, especially when only 2-3 tags are visible.
    private static final double OUTLIER_REJECT_DISTANCE_INCHES = 6.0;

    // Beyond this range, a tag's positional accuracy degrades enough that it
    // should count less toward the fused pose, even if decisionMargin says
    // the detector is confident it found the tag - margin measures "did we
    // find the tag correctly," not "is the resulting pose still precise at
    // this distance." TUNE this against your actual field/tag size.
    private static final double RELIABLE_RANGE_INCHES = 48.0;

    private HivePoseEstimator() {
    }

    public static List<HiveCell> estimate(TagClusterMapper.FilterResult filtered) {
        List<HiveCell> cells = new ArrayList<>();
        if (filtered == null) {
            return cells;
        }
        for (HiveCluster cluster : HiveCluster.values()) {
            List<AprilTagDetection> tags = filtered.tagsFor(cluster);
            if (!tags.isEmpty()) {
                cells.add(estimateCluster(cluster, tags));
            }
        }
        return cells;
    }

    public static HiveCell estimateCluster(HiveCluster cluster, List<AprilTagDetection> tags) {
        int[] visibleIds = new int[tags.size()];
        for (int i = 0; i < tags.size(); i++) {
            visibleIds[i] = tags.get(i).id;
        }
        Arrays.sort(visibleIds);

        double marginSum = 0;

        // First pass: collect every tag with a usable pose. We need the
        // full set before we can compute a median for outlier rejection.
        List<AprilTagDetection> withPose = new ArrayList<>();
        for (AprilTagDetection tag : tags) {
            marginSum += tag.decisionMargin;
            if (tag.metadata == null || tag.ftcPose == null) {
                continue;
            }
            AprilTagPoseFtc pose = tag.ftcPose;
            if (!Double.isFinite(pose.x) || !Double.isFinite(pose.y) || !Double.isFinite(pose.z)) {
                continue;
            }
            withPose.add(tag);
        }

        double meanMargin = tags.isEmpty() ? 0 : marginSum / tags.size();

        // Median x/y across all pose-bearing tags, used only to detect a
        // tag whose reading is far from where the rest agree it should be.
        // With 1-2 tags there's nothing meaningful to reject against, so
        // outlier filtering only kicks in at 3+.
        double medianX = Double.NaN;
        double medianY = Double.NaN;
        if (withPose.size() >= 3) {
            double[] xs = new double[withPose.size()];
            double[] ys = new double[withPose.size()];
            for (int i = 0; i < withPose.size(); i++) {
                xs[i] = withPose.get(i).ftcPose.x;
                ys[i] = withPose.get(i).ftcPose.y;
            }
            Arrays.sort(xs);
            Arrays.sort(ys);
            medianX = xs[xs.length / 2];
            medianY = ys[ys.length / 2];
        }

        double weightSum = 0;
        double xSum = 0;
        double ySum = 0;
        double zSum = 0;
        int poseTags = 0;

        double yawSin = 0;
        double yawCos = 0;
        double pitchSin = 0;
        double pitchCos = 0;
        double rollSin = 0;
        double rollCos = 0;

        int facingUpVotes = 0;
        int facingDownVotes = 0;

        for (AprilTagDetection tag : withPose) {
            AprilTagPoseFtc pose = tag.ftcPose;

            // Outlier rejection - skip a tag whose position disagrees with
            // the cluster median by more than the threshold. Only active
            // when we have enough tags (3+) that a median is meaningful.
            if (Double.isFinite(medianX) && Double.isFinite(medianY)) {
                double distFromMedian = Math.hypot(pose.x - medianX, pose.y - medianY);
                if (distFromMedian > OUTLIER_REJECT_DISTANCE_INCHES) {
                    continue;
                }
            }

            // decisionMargin is the detector's own quality score. A clean tag
            // pulls the fused point harder than a tag that barely decoded.
            double marginWeight = Math.max(tag.decisionMargin, 1.0);

            // Range-based down-weighting - a distant tag's pose is less
            // precise even at high decisionMargin, so it should count less
            // than a close tag with the same margin. Falls off linearly
            // past RELIABLE_RANGE_INCHES rather than being hard-cut, so a
            // single far tag still contributes something when it's all
            // that's visible.
            double range = pose.range;
            double rangeFactor = (Double.isFinite(range) && range > RELIABLE_RANGE_INCHES)
                    ? RELIABLE_RANGE_INCHES / range
                    : 1.0;

            double weight = marginWeight * rangeFactor;
            weightSum += weight;
            xSum += weight * pose.x;
            ySum += weight * pose.y;
            zSum += weight * pose.z;
            poseTags++;

            yawSin += weight * Math.sin(Math.toRadians(pose.yaw));
            yawCos += weight * Math.cos(Math.toRadians(pose.yaw));
            pitchSin += weight * Math.sin(Math.toRadians(pose.pitch));
            pitchCos += weight * Math.cos(Math.toRadians(pose.pitch));
            rollSin += weight * Math.sin(Math.toRadians(pose.roll));
            rollCos += weight * Math.cos(Math.toRadians(pose.roll));

            CellOrientation vote = classifyRoll(pose.roll);
            if (vote == CellOrientation.FACING_UP) {
                facingUpVotes++;
            } else if (vote == CellOrientation.FACING_DOWN) {
                facingDownVotes++;
            }
        }

        if (poseTags == 0 || weightSum <= 0) {
            return HiveCell.detected(
                    cluster,
                    visibleIds,
                    0,
                    false,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    meanMargin,
                    CellOrientation.UNKNOWN,
                    TargetConfidence.UNKNOWN,
                    score(false, visibleIds.length, meanMargin, CellOrientation.UNKNOWN));
        }

        // Camera frame -> robot center. ftcPose is measured at the lens.
        double relativeX = (xSum / weightSum) + BioBuzzVisionConfig.CAMERA_RIGHT_OF_CENTER_INCHES;
        double relativeY = (ySum / weightSum)
                + BioBuzzVisionConfig.CAMERA_FORWARD_OF_CENTER_INCHES
                + BioBuzzVisionConfig.OPENING_FORWARD_OF_TAGS_INCHES;
        double relativeZ = (zSum / weightSum) + BioBuzzVisionConfig.OPENING_ABOVE_TAGS_INCHES;

        double yaw = circularMeanDegrees(yawSin, yawCos);
        double pitch = circularMeanDegrees(pitchSin, pitchCos);
        double roll = circularMeanDegrees(rollSin, rollCos);

        CellOrientation orientation = combineOrientation(roll, facingUpVotes, facingDownVotes);
        TargetConfidence confidence = confidence(poseTags, meanMargin, true);
        double distance = getDistanceToTarget(relativeX, relativeY);
        double bearing = bearingDegrees(relativeX, relativeY);

        return HiveCell.detected(
                cluster,
                visibleIds,
                poseTags,
                true,
                relativeX,
                relativeY,
                relativeZ,
                distance,
                bearing,
                yaw,
                pitch,
                roll,
                Math.hypot(distance, relativeZ),
                meanMargin,
                orientation,
                confidence,
                score(true, poseTags, meanMargin, orientation));
    }

    /**
     * Horizontal distance from the robot center to the target point, inches.
     * {@code relativeX} is right and {@code relativeY} is forward, both taken
     * from the AprilTag translation. Pixel size is not involved.
     */
    public static double getDistanceToTarget(double relativeXInches, double relativeYInches) {
        return Math.hypot(relativeXInches, relativeYInches);
    }

    /**
     * Degrees, positive when the target is to the robot's left.
     * That is the same sign as {@code ftcPose.bearing} (positive bearing
     * means turn counterclockwise), so X-right has to be negated.
     */
    public static double bearingDegrees(double relativeXInches, double relativeYInches) {
        if (!Double.isFinite(relativeXInches) || !Double.isFinite(relativeYInches)) {
            return Double.NaN;
        }
        if (Math.hypot(relativeXInches, relativeYInches) < 0.5) {
            return Double.NaN;
        }
        return Math.toDegrees(Math.atan2(-relativeXInches, relativeYInches));
    }

    static CellOrientation classifyRoll(double rollDegrees) {
        if (!Double.isFinite(rollDegrees)) {
            return CellOrientation.UNKNOWN;
        }
        double absRoll = Math.abs(normalizeDegrees(rollDegrees));
        // Strict inequalities leave exactly 90 degrees as UNKNOWN (the cell is mid-tip).
        if (absRoll < BioBuzzVisionConfig.FACING_UP_MAX_ROLL_DEG) {
            return CellOrientation.FACING_UP;
        }
        if (absRoll > BioBuzzVisionConfig.FACING_DOWN_MIN_ROLL_DEG) {
            return CellOrientation.FACING_DOWN;
        }
        return CellOrientation.UNKNOWN;
    }

    /**
     * If two tags on the same cell disagree about up versus down, the cell
     * is UNKNOWN. Averaging those rolls would land near the tipping point
     * and look confident.
     */
    private static CellOrientation combineOrientation(double meanRoll, int upVotes, int downVotes) {
        if (upVotes > 0 && downVotes > 0) {
            return CellOrientation.UNKNOWN;
        }
        return classifyRoll(meanRoll);
    }

    static TargetConfidence confidence(int poseTagCount, double meanDecisionMargin, boolean poseValid) {
        if (!poseValid || poseTagCount <= 0) {
            return TargetConfidence.UNKNOWN;
        }
        if (poseTagCount >= BioBuzzVisionConfig.HIGH_MIN_TAGS
                && meanDecisionMargin >= BioBuzzVisionConfig.HIGH_MIN_DECISION_MARGIN) {
            return TargetConfidence.HIGH;
        }
        if (poseTagCount >= BioBuzzVisionConfig.MEDIUM_MIN_TAGS
                && meanDecisionMargin >= BioBuzzVisionConfig.MEDIUM_MIN_DECISION_MARGIN) {
            return TargetConfidence.MEDIUM;
        }
        return TargetConfidence.LOW;
    }

    /**
     * Higher is a better lock. Facing-up is worth more than one extra tag,
     * so the downward cell of the same HIVE does not win just because more
     * of its sticker happens to be in frame.
     */
    static double score(boolean poseValid, int poseTagCount, double meanDecisionMargin, CellOrientation orientation) {
        double score = poseTagCount * 100.0 + meanDecisionMargin;
        if (!poseValid) {
            return score;
        }
        score += 50.0;
        if (orientation == CellOrientation.FACING_UP) {
            score += 250.0;
        } else if (orientation == CellOrientation.FACING_DOWN) {
            score -= 100.0;
        }
        return score;
    }

    private static double circularMeanDegrees(double sinSum, double cosSum) {
        return Math.toDegrees(Math.atan2(sinSum, cosSum));
    }

    private static double normalizeDegrees(double degrees) {
        double wrapped = degrees % 360.0;
        if (wrapped > 180.0) {
            wrapped -= 360.0;
        } else if (wrapped < -180.0) {
            wrapped += 360.0;
        }
        return wrapped;
    }
}
