package org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz;

/**
 * One HIVE CELL as seen this frame. Built only from tags that belong to
 * that cell; the four printed ids stay one object.
 *
 * <p>Translation is robot-center relative, inches, FTC camera axes after the
 * camera mount offset is applied: X right, Y forward, Z up.
 */
public final class HiveCell {

    public final HiveCluster cluster;
    public final Alliance alliance;
    public final FieldSide fieldSide;
    public final int[] tagIds;

    public final boolean detected;
    public final int[] visibleTagIds;
    public final int visibleTagCount;
    public final int poseTagCount;

    public final boolean poseValid;
    public final double relativeX;
    public final double relativeY;
    public final double relativeZ;
    public final double distance;
    public final double bearing;
    public final double yaw;
    public final double pitch;
    public final double roll;
    public final double range;
    public final double meanDecisionMargin;

    public final CellOrientation orientation;
    public final TargetConfidence confidence;
    public final double score;

    private HiveCell(HiveCluster cluster,
                      boolean detected,
                      int[] visibleTagIds,
                      int poseTagCount,
                      boolean poseValid,
                      double relativeX,
                      double relativeY,
                      double relativeZ,
                      double distance,
                      double bearing,
                      double yaw,
                      double pitch,
                      double roll,
                      double range,
                      double meanDecisionMargin,
                      CellOrientation orientation,
                      TargetConfidence confidence,
                      double score) {
        this.cluster = cluster;
        this.alliance = cluster.alliance;
        this.fieldSide = cluster.fieldSide;
        this.tagIds = cluster.tagIds;
        this.detected = detected;
        this.visibleTagIds = visibleTagIds;
        this.visibleTagCount = visibleTagIds.length;
        this.poseTagCount = poseTagCount;
        this.poseValid = poseValid;
        this.relativeX = relativeX;
        this.relativeY = relativeY;
        this.relativeZ = relativeZ;
        this.distance = distance;
        this.bearing = bearing;
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
        this.range = range;
        this.meanDecisionMargin = meanDecisionMargin;
        this.orientation = orientation;
        this.confidence = confidence;
        this.score = score;
    }

    public static HiveCell undetected(HiveCluster cluster) {
        return new HiveCell(
                cluster,
                false,
                new int[0],
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
                0,
                CellOrientation.UNKNOWN,
                TargetConfidence.UNKNOWN,
                -1);
    }

    public static HiveCell detected(HiveCluster cluster,
                                     int[] visibleTagIds,
                                     int poseTagCount,
                                     boolean poseValid,
                                     double relativeX,
                                     double relativeY,
                                     double relativeZ,
                                     double distance,
                                     double bearing,
                                     double yaw,
                                     double pitch,
                                     double roll,
                                     double range,
                                     double meanDecisionMargin,
                                     CellOrientation orientation,
                                     TargetConfidence confidence,
                                     double score) {
        return new HiveCell(
                cluster,
                true,
                visibleTagIds,
                poseTagCount,
                poseValid,
                relativeX,
                relativeY,
                relativeZ,
                distance,
                bearing,
                yaw,
                pitch,
                roll,
                range,
                meanDecisionMargin,
                orientation,
                confidence,
                score);
    }
}
