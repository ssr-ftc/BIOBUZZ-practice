package org.firstinspires.ftc.teamcode.OFSB1.Vision.biobuzz;

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

        double weightSum = 0;
        double xSum = 0;
        double ySum = 0;
        double zSum = 0;
        double marginSum = 0;
        int poseTags = 0;

        double yawSin = 0;
        double yawCos = 0;
        double pitchSin = 0;
        double pitchCos = 0;
        double rollSin = 0;
        double rollCos = 0;

        int facingUpVotes = 0;
        int facingDownVotes = 0;

        for (AprilTagDetection tag : tags) {
            marginSum += tag.decisionMargin;
            if (tag.metadata == null || tag.ftcPose == null) {
                continue;
            }
            AprilTagPoseFtc pose = tag.ftcPose;
            if (!Double.isFinite(pose.x) || !Double.isFinite(pose.y) || !Double.isFinite(pose.z)) {
                continue;
            }

            // decisionMargin is the detector's own quality score. A clean tag
            // pulls the fused point harder than a tag that barely decoded.
            double weight = Math.max(tag.decisionMargin, 1.0);
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

        double meanMargin = tags.isEmpty() ? 0 : marginSum / tags.size();
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
        // Strict inequalities leave exactly 90° as UNKNOWN (the cell is mid-tip).
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
