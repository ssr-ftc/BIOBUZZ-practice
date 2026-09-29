package org.firstinspires.ftc.teamcode.OFSB1.Vision.biobuzz;

import java.util.List;

/**
 * Picks one alliance CELL and refuses to flip to another cell on a single
 * noisy frame. Does not drive.
 */
public final class TargetSelector {

    private HiveCluster lockedCluster;
    private double lockedScore;
    private ShootingTarget heldValid;
    private double lostSinceSeconds = -1;
    private boolean everLocked;

    public void reset() {
        lockedCluster = null;
        lockedScore = 0;
        heldValid = null;
        lostSinceSeconds = -1;
        everLocked = false;
    }

    public VisionState select(List<HiveCell> cells,
                               TagClusterMapper.FilterResult filtered,
                               Alliance alliance,
                               boolean cameraStreaming,
                               double nowSeconds) {
        if (alliance == null) {
            reset();
            return VisionState.empty(TargetStatus.NO_TARGET, TargetBlockReason.ALLIANCE_NOT_SELECTED);
        }
        if (!cameraStreaming) {
            return holdOrClear(nowSeconds, TargetBlockReason.CAMERA_NOT_READY, null);
        }
        if (cells == null || cells.isEmpty()) {
            TargetBlockReason reason = TargetBlockReason.NO_RELEVANT_TAGS;
            if (filtered != null && filtered.opposingTagCount > 0) {
                reason = TargetBlockReason.OPPOSING_ALLIANCE_ONLY;
            }
            return holdOrClear(nowSeconds, reason, null);
        }

        HiveCell best = bestOf(cells);
        HiveCell lockedVisible = find(cells, lockedCluster);

        if (lockedVisible != null) {
            HiveCell chosen = lockedVisible;
            if (best.cluster != lockedVisible.cluster
                    && best.score >= lockedVisible.score + BioBuzzVisionConfig.TARGET_SWITCH_MARGIN) {
                chosen = best;
            }
            return publish(chosen);
        }

        // Locked cell dropped out. A different cell only takes over immediately
        // when it is clearly stronger. A similar score is treated as flicker
        // and the previous target is held until the timeout.
        if (lockedCluster != null
                && best.score < lockedScore + BioBuzzVisionConfig.TARGET_SWITCH_MARGIN) {
            return holdOrClear(nowSeconds, TargetBlockReason.NO_RELEVANT_TAGS, best);
        }

        return publish(best);
    }

    private HiveCell bestOf(List<HiveCell> cells) {
        HiveCell best = null;
        for (HiveCell cell : cells) {
            if (best == null || cell.score > best.score) {
                best = cell;
            }
        }
        return best;
    }

    private static HiveCell find(List<HiveCell> cells, HiveCluster cluster) {
        if (cluster == null) {
            return null;
        }
        for (HiveCell cell : cells) {
            if (cell.cluster == cluster) {
                return cell;
            }
        }
        return null;
    }

    private VisionState publish(HiveCell chosen) {
        lostSinceSeconds = -1;
        lockedCluster = chosen.cluster;
        lockedScore = chosen.score;

        Classification classification = classify(chosen);
        if (classification.status == TargetStatus.TARGET_VALID) {
            everLocked = true;
            heldValid = classification.target;
        }
        return new VisionState(classification.status, classification.target, false, classification.reason);
    }

    /**
     * @param fallback cell to accept once the hold expires. Null when the
     *                 frame has nothing worth switching to.
     */
    private VisionState holdOrClear(double nowSeconds, TargetBlockReason absentReason, HiveCell fallback) {
        if (heldValid != null && heldValid.valid) {
            if (lostSinceSeconds < 0) {
                lostSinceSeconds = nowSeconds;
            }
            double heldFor = nowSeconds - lostSinceSeconds;
            if (heldFor <= BioBuzzVisionConfig.LOST_TARGET_TIMEOUT_SECONDS) {
                ShootingTarget held = new ShootingTarget(
                        false,
                        false,
                        heldValid.cell,
                        heldValid.distance,
                        heldValid.bearing,
                        heldValid.relativeX,
                        heldValid.relativeY,
                        heldValid.relativeZ,
                        heldValid.orientation,
                        heldValid.confidence,
                        TargetBlockReason.TEMPORARILY_LOST);
                return new VisionState(TargetStatus.TEMPORARILY_LOST, held, true, TargetBlockReason.TEMPORARILY_LOST);
            }
        }
        lockedCluster = null;
        lockedScore = 0;
        heldValid = null;
        lostSinceSeconds = -1;
        if (fallback != null) {
            return publish(fallback);
        }
        TargetStatus status = everLocked ? TargetStatus.NO_TARGET : TargetStatus.SEARCHING;
        return VisionState.empty(status, absentReason);
    }

    private Classification classify(HiveCell cell) {
        if (!cell.poseValid) {
            return new Classification(
                    TargetStatus.TARGET_INVALID,
                    ShootingTarget.fromCell(cell, false, false, TargetBlockReason.POSE_UNAVAILABLE),
                    TargetBlockReason.POSE_UNAVAILABLE);
        }
        if (cell.orientation == CellOrientation.FACING_DOWN) {
            return new Classification(
                    TargetStatus.TARGET_INVALID,
                    ShootingTarget.fromCell(cell, false, false, TargetBlockReason.FACING_DOWN),
                    TargetBlockReason.FACING_DOWN);
        }
        if (cell.orientation != CellOrientation.FACING_UP) {
            return new Classification(
                    TargetStatus.TARGET_INVALID,
                    ShootingTarget.fromCell(cell, false, false, TargetBlockReason.ORIENTATION_UNKNOWN),
                    TargetBlockReason.ORIENTATION_UNKNOWN);
        }
        if (cell.meanDecisionMargin < BioBuzzVisionConfig.MIN_MARGIN_FOR_ORIENTATION) {
            return new Classification(
                    TargetStatus.LOW_CONFIDENCE,
                    ShootingTarget.fromCell(cell, false, false, TargetBlockReason.LOW_CONFIDENCE),
                    TargetBlockReason.LOW_CONFIDENCE);
        }

        // Facing up is enough. Bearing, yaw, and distance describe where the
        // robot is standing on this side. They do not change the cell's tilt.
        return new Classification(
                TargetStatus.TARGET_VALID,
                ShootingTarget.fromCell(cell, true, true, TargetBlockReason.NONE),
                TargetBlockReason.NONE);
    }

    /**
     * True when this same-alliance cell is visibly facing up.
     * Position along the audience side or the scoring side does not matter.
     * The camera does have to see at least one tag on that cell.
     */
    public static boolean isShootable(HiveCell cell) {
        return cell != null
                && cell.detected
                && cell.poseValid
                && cell.poseTagCount >= 1
                && cell.orientation == CellOrientation.FACING_UP
                && cell.meanDecisionMargin >= BioBuzzVisionConfig.MIN_MARGIN_FOR_ORIENTATION;
    }

    private static final class Classification {
        final TargetStatus status;
        final ShootingTarget target;
        final TargetBlockReason reason;

        Classification(TargetStatus status, ShootingTarget target, TargetBlockReason reason) {
            this.status = status;
            this.target = target;
            this.reason = reason;
        }
    }
}
