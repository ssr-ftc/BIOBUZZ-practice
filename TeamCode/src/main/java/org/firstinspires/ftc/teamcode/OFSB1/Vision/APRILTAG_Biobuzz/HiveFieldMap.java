package org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz;

import org.firstinspires.ftc.easyatl.FieldTags;

/**
 * BIOBUZZ HIVE tags in EasyATL's field map. EasyATL 1.2.2 ships DECODE tags
 * only, so this set is built with {@link FieldTags#custom(String)}.
 *
 * <p>The four ids on one sticker share that cell's center. They are one
 * landmark for localization. Shooting still treats them as one cell elsewhere.
 */
public final class HiveFieldMap {

    private HiveFieldMap() {
    }

    public static FieldTags tags() {
        double half = BioBuzzVisionConfig.CELL_OFFSET_FROM_PIVOT_INCHES;
        FieldTags.Builder builder = FieldTags.custom("BIOBUZZ");
        // Audience cells face +X (toward the audience). Opposite cells face -X.
        addCell(builder, HiveCluster.RED_AUDIENCE, half, BioBuzzVisionConfig.RED_HIVE_Y_INCHES, 0.0);
        addCell(builder, HiveCluster.RED_OPPOSITE, -half, BioBuzzVisionConfig.RED_HIVE_Y_INCHES, Math.PI);
        addCell(builder, HiveCluster.BLUE_AUDIENCE, half, BioBuzzVisionConfig.BLUE_HIVE_Y_INCHES, 0.0);
        addCell(builder, HiveCluster.BLUE_OPPOSITE, -half, BioBuzzVisionConfig.BLUE_HIVE_Y_INCHES, Math.PI);
        return builder.build();
    }

    private static void addCell(FieldTags.Builder builder, HiveCluster cell,
                                 double xInches, double yInches, double facingRadians) {
        for (int id : cell.tagIds) {
            builder.add(id, xInches, yInches, facingRadians, cell.displayName());
        }
    }
}
