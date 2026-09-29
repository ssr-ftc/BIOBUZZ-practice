package org.firstinspires.ftc.teamcode.OFSB1.Vision.biobuzz;

/**
 * Robot-relative shooting solution for one CELL. Vision fills this in.
 * TeleOp decides whether to aim or shoot.
 */
public final class ShootingTarget {

    public static final ShootingTarget NONE = new ShootingTarget(
            false,
            false,
            null,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            CellOrientation.UNKNOWN,
            TargetConfidence.UNKNOWN,
            TargetBlockReason.NO_RELEVANT_TAGS);

    public final boolean valid;
    public final boolean shootable;
    public final HiveCell cell;
    public final double distance;
    public final double bearing;
    public final double relativeX;
    public final double relativeY;
    public final double relativeZ;
    public final CellOrientation orientation;
    public final TargetConfidence confidence;
    public final TargetBlockReason blockReason;

    public ShootingTarget(boolean valid,
                           boolean shootable,
                           HiveCell cell,
                           double distance,
                           double bearing,
                           double relativeX,
                           double relativeY,
                           double relativeZ,
                           CellOrientation orientation,
                           TargetConfidence confidence,
                           TargetBlockReason blockReason) {
        this.valid = valid;
        this.shootable = shootable;
        this.cell = cell;
        this.distance = distance;
        this.bearing = bearing;
        this.relativeX = relativeX;
        this.relativeY = relativeY;
        this.relativeZ = relativeZ;
        this.orientation = orientation == null ? CellOrientation.UNKNOWN : orientation;
        this.confidence = confidence == null ? TargetConfidence.UNKNOWN : confidence;
        this.blockReason = blockReason == null ? TargetBlockReason.NONE : blockReason;
    }

    public static ShootingTarget fromCell(HiveCell cell, boolean valid, boolean shootable, TargetBlockReason reason) {
        if (cell == null || !cell.detected) {
            return NONE;
        }
        return new ShootingTarget(
                valid,
                shootable,
                cell,
                cell.distance,
                cell.bearing,
                cell.relativeX,
                cell.relativeY,
                cell.relativeZ,
                cell.orientation,
                cell.confidence,
                reason);
    }

    public double getDistance() {
        return distance;
    }

    public double getBearing() {
        return bearing;
    }

    public boolean isShootable() {
        return shootable;
    }

    public HiveCell getCell() {
        return cell;
    }
}
