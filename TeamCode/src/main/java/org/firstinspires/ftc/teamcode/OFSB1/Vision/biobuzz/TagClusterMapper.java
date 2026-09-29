package org.firstinspires.ftc.teamcode.OFSB1.Vision.biobuzz;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/**
 * Maps a raw tag id to one HIVE CELL, and groups a frame's detections by cell.
 * Ids outside 30–45, and ids on the other alliance, never become targets.
 */
public final class TagClusterMapper {

    private static final HiveCluster[] BY_ID = new HiveCluster[46];

    static {
        for (HiveCluster cluster : HiveCluster.values()) {
            for (int id : cluster.tagIds) {
                BY_ID[id] = cluster;
            }
        }
    }

    private TagClusterMapper() {
    }

    /**
     * CELL that owns {@code tagId}, or null when the id is not a BIOBUZZ HIVE tag.
     * The returned value is the cluster identity (four ids, one physical cell),
     * not a live pose.
     */
    public static HiveCluster getCellForTagId(int tagId) {
        if (tagId < 0 || tagId >= BY_ID.length) {
            return null;
        }
        return BY_ID[tagId];
    }

    public static AprilTagLibrary createTagLibrary() {
        AprilTagLibrary.Builder builder = new AprilTagLibrary.Builder();
        for (HiveCluster cluster : HiveCluster.values()) {
            for (int id : cluster.tagIds) {
                builder.addTag(
                        id,
                        cluster.displayName(),
                        BioBuzzVisionConfig.TAG_SIZE_INCHES,
                        DistanceUnit.INCH);
            }
        }
        return builder.build();
    }

    public static FilterResult filter(List<AprilTagDetection> detections, Alliance alliance) {
        Map<HiveCluster, List<AprilTagDetection>> grouped = new EnumMap<>(HiveCluster.class);
        int unknown = 0;
        int opposing = 0;
        int rejectedHamming = 0;

        if (detections != null && alliance != null) {
            for (AprilTagDetection detection : detections) {
                if (detection == null) {
                    continue;
                }
                if (detection.hamming > BioBuzzVisionConfig.MAX_HAMMING) {
                    rejectedHamming++;
                    continue;
                }
                HiveCluster cluster = getCellForTagId(detection.id);
                if (cluster == null) {
                    unknown++;
                    continue;
                }
                if (cluster.alliance != alliance) {
                    opposing++;
                    continue;
                }
                List<AprilTagDetection> members = grouped.get(cluster);
                if (members == null) {
                    members = new ArrayList<>();
                    grouped.put(cluster, members);
                }
                replaceIfBetter(members, detection);
            }
        }

        return new FilterResult(grouped, unknown, opposing, rejectedHamming);
    }

    /** One detection per id. If the same id appears twice, keep the cleaner decode. */
    private static void replaceIfBetter(List<AprilTagDetection> members, AprilTagDetection detection) {
        for (int i = 0; i < members.size(); i++) {
            if (members.get(i).id == detection.id) {
                if (detection.decisionMargin > members.get(i).decisionMargin) {
                    members.set(i, detection);
                }
                return;
            }
        }
        members.add(detection);
    }

    public static final class FilterResult {
        public final Map<HiveCluster, List<AprilTagDetection>> clusters;
        public final int unknownTagCount;
        public final int opposingTagCount;
        public final int rejectedHammingCount;

        FilterResult(Map<HiveCluster, List<AprilTagDetection>> clusters,
                     int unknownTagCount,
                     int opposingTagCount,
                     int rejectedHammingCount) {
            this.clusters = clusters;
            this.unknownTagCount = unknownTagCount;
            this.opposingTagCount = opposingTagCount;
            this.rejectedHammingCount = rejectedHammingCount;
        }

        public boolean isEmpty() {
            return clusters.isEmpty();
        }

        public List<AprilTagDetection> tagsFor(HiveCluster cluster) {
            List<AprilTagDetection> tags = clusters.get(cluster);
            return tags == null ? Collections.<AprilTagDetection>emptyList() : tags;
        }
    }
}
