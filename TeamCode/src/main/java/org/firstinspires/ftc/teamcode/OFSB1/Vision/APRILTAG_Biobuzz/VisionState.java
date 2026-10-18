package org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz;

/**
 * What TeleOp is allowed to read after each vision update.
 * A held target during {@link TargetStatus#TEMPORARILY_LOST} is still
 * returned by {@link #getTarget()}, but it is not valid and not shootable.
 */
public final class VisionState {

    private final TargetStatus status;
    private final ShootingTarget target;
    private final boolean holdingLast;
    private final TargetBlockReason reason;

    public VisionState(TargetStatus status,
                        ShootingTarget target,
                        boolean holdingLast,
                        TargetBlockReason reason) {
        this.status = status == null ? TargetStatus.NO_TARGET : status;
        this.target = target == null ? ShootingTarget.NONE : target;
        this.holdingLast = holdingLast;
        this.reason = reason == null ? TargetBlockReason.NONE : reason;
    }

    public static VisionState empty(TargetStatus status, TargetBlockReason reason) {
        return new VisionState(status, ShootingTarget.NONE, false, reason);
    }

    public TargetStatus getStatus() {
        return status;
    }

    public TargetBlockReason getReason() {
        return reason;
    }

    public boolean isHoldingLast() {
        return holdingLast;
    }

    /** A live alliance cell, or the cell we are briefly holding through a dropout. */
    public boolean hasTarget() {
        return target.cell != null && (target.cell.detected || holdingLast);
    }

    /** Live, upward, confident cell. False while the target is only being held. */
    public boolean hasValidTarget() {
        return status == TargetStatus.TARGET_VALID && target.valid && !holdingLast;
    }

    public ShootingTarget getTarget() {
        return target;
    }

    public ShootingTarget getRecommendedTarget() {
        return hasValidTarget() ? target : ShootingTarget.NONE;
    }

    public double getTargetDistance() {
        return hasTarget() ? target.distance : Double.NaN;
    }

    public double getTargetBearing() {
        return hasTarget() ? target.bearing : Double.NaN;
    }

    public boolean isTargetShootable() {
        return hasValidTarget() && target.shootable;
    }

    public TargetConfidence getTargetConfidence() {
        return hasTarget() ? target.confidence : TargetConfidence.UNKNOWN;
    }
}
