package org.firstinspires.ftc.teamcode.OFSB1.Vision.biobuzz;

/**
 * One physical HIVE CELL, identified by its four AprilTags.
 * The four ids are not four targets.
 */
public enum HiveCluster {
    RED_OPPOSITE(Alliance.RED, FieldSide.OPPOSITE, 30, 31, 32, 33),
    RED_AUDIENCE(Alliance.RED, FieldSide.AUDIENCE, 34, 35, 36, 37),
    BLUE_AUDIENCE(Alliance.BLUE, FieldSide.AUDIENCE, 38, 39, 40, 41),
    BLUE_OPPOSITE(Alliance.BLUE, FieldSide.OPPOSITE, 42, 43, 44, 45);

    public final Alliance alliance;
    public final FieldSide fieldSide;
    public final int[] tagIds;

    HiveCluster(Alliance alliance, FieldSide fieldSide, int... tagIds) {
        this.alliance = alliance;
        this.fieldSide = fieldSide;
        this.tagIds = tagIds;
    }

    public int tagCount() {
        return tagIds.length;
    }

    public boolean contains(int tagId) {
        for (int id : tagIds) {
            if (id == tagId) {
                return true;
            }
        }
        return false;
    }

    /** Driver-station label, for example "RED AUDIENCE". */
    public String displayName() {
        return alliance.name() + " " + fieldSide.name();
    }
}
