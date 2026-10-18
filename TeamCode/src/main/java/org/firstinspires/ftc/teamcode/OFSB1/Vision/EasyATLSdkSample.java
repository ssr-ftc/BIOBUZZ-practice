package org.firstinspires.ftc.teamcode.OFSB1.Vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.easyatl.DefaultSdkConstants;
import org.firstinspires.ftc.easyatl.FieldPose;
import org.firstinspires.ftc.easyatl.FtcEasyATL;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "BIOBUZZ AprilTag Test", group = "BIOBUZZ")
public class EasyATLSdkSample extends OpMode {

    private static final boolean APPLY_VISION_CORRECTION = false;

    private AprilTagProcessor processor;
    private VisionPortal portal;
    private FtcEasyATL localizer;

    // =========================================================
    // BIOBUZZ ENUMS
    // =========================================================

    private enum Alliance {
        RED,
        BLUE,
        UNKNOWN
    }

    private enum CellSide {
        AUDIENCE,
        OPPOSITE_AUDIENCE,
        UNKNOWN
    }

    // =========================================================
    // INITIALIZATION
    // =========================================================

    @Override
    public void init() {

        processor = DefaultSdkConstants.createProcessor();

        portal = DefaultSdkConstants.createPortal(
                hardwareMap,
                processor,
                telemetry
        );

        localizer = DefaultSdkConstants.createLocalizer(
                DefaultSdkConstants.config()
        );

        /*
         * IMPORTANT:
         *
         * EasyATL's built-in "latest season" may still contain
         * older season tag definitions.
         *
         * We want the FTC SDK's CURRENT game tag database.
         */
        localizer.addCurrentGameTags();

        telemetry.addLine("BIOBUZZ AprilTag Vision Ready");
        telemetry.addLine("Waiting for tags...");
        telemetry.update();
    }

    // =========================================================
    // MAIN LOOP
    // =========================================================

    @Override
    public void loop() {

        // Give all current FTC AprilTag detections to EasyATL.
        boolean accepted =
                localizer.localize(processor.getDetections());

        telemetry.addLine("========== BIOBUZZ VISION ==========");

        // -----------------------------------------------------
        // EASYATL ROBOT FIELD POSE
        // -----------------------------------------------------

        if (localizer.hasPose()) {

            FieldPose vision = localizer.getPose();

            telemetry.addData(
                    "Robot Position",
                    "X %.1f   Y %.1f",
                    vision.x,
                    vision.y
            );

            telemetry.addData(
                    "Robot Heading",
                    "%.1f deg",
                    vision.headingDegrees()
            );

            telemetry.addData(
                    "EasyATL Quality",
                    "%.0f%%",
                    100 * localizer.getQuality()
            );

            telemetry.addData(
                    "EasyATL Visible",
                    localizer.getVisibleTags()
            );

            telemetry.addData(
                    "EasyATL Accepted",
                    localizer.getAcceptedTags()
            );

        } else {

            telemetry.addLine("Robot Pose: NOT AVAILABLE");
        }

        telemetry.addLine("");
        telemetry.addLine("---------- HIVE STATE ----------");

        // -----------------------------------------------------
        // BIOBUZZ APRILTAG DETECTION
        // -----------------------------------------------------

        boolean foundBioBuzzTag = false;

        for (AprilTagDetection detection :
                processor.getDetections()) {

            int tagId = detection.id;

            // Ignore tags that are not BIOBUZZ HIVE tags.
            if (!isBioBuzzHiveTag(tagId)) {
                continue;
            }

            foundBioBuzzTag = true;

            Alliance alliance =
                    getAlliance(tagId);

            CellSide downCell =
                    getDownCell(tagId);

            CellSide upCell =
                    getUpCell(tagId);

            telemetry.addData(
                    "Visible Tag",
                    tagId
            );

            telemetry.addData(
                    "Alliance",
                    alliance
            );

            telemetry.addData(
                    "CELL DOWN",
                    downCell
            );

            telemetry.addData(
                    ">>> SHOOT AT CELL",
                    upCell
            );

            // -------------------------------------------------
            // RAW CAMERA -> APRILTAG INFORMATION
            // -------------------------------------------------

            if (detection.ftcPose != null) {

                telemetry.addData(
                        "Tag Range",
                        "%.1f in",
                        detection.ftcPose.range
                );

                telemetry.addData(
                        "Tag Bearing",
                        "%.1f deg",
                        detection.ftcPose.bearing
                );

                telemetry.addData(
                        "Tag Yaw",
                        "%.1f deg",
                        detection.ftcPose.yaw
                );

                /*
                 * Bearing tells us where the detected tag is
                 * relative to the CAMERA.
                 *
                 * Negative bearing = one direction
                 * Positive bearing = opposite direction.
                 *
                 * We'll verify the sign on the physical robot
                 * before using it for automatic turning.
                 */
                telemetry.addData(
                        "Angle Robot -> Tag",
                        "%.1f deg",
                        detection.ftcPose.bearing
                );

                telemetry.addData(
                        "Distance Robot -> Tag",
                        "%.1f in",
                        detection.ftcPose.range
                );
            }

            telemetry.addLine("-------------------------------");
        }

        if (!foundBioBuzzTag) {
            telemetry.addLine("No BIOBUZZ HIVE tag visible");
        }

        telemetry.addLine("");
        telemetry.addLine("---------- DEBUG ----------");

        telemetry.addData(
                "Raw Tag Count",
                processor.getDetections().size()
        );

        telemetry.addData(
                "New EasyATL Measurement",
                accepted
        );

        telemetry.update();
    }

    // =========================================================
    // BIOBUZZ TAG FUNCTIONS
    // =========================================================

    /**
     * BIOBUZZ HIVE AprilTags are IDs 30 through 45.
     */
    private boolean isBioBuzzHiveTag(int tagId) {

        return tagId >= 30 && tagId <= 45;
    }

    /**
     * Determine which alliance HIVE this tag belongs to.
     *
     * RED:
     * 30-37
     *
     * BLUE:
     * 38-45
     */
    private Alliance getAlliance(int tagId) {

        if (tagId >= 30 && tagId <= 37) {
            return Alliance.RED;
        }

        if (tagId >= 38 && tagId <= 45) {
            return Alliance.BLUE;
        }

        return Alliance.UNKNOWN;
    }

    /**
     * Determine which CELL is currently facing DOWN.
     *
     * BIOBUZZ tag groups:
     *
     * 30-33 = RED opposite-audience CELL
     * 34-37 = RED audience CELL
     *
     * 38-41 = BLUE audience CELL
     * 42-45 = BLUE opposite-audience CELL
     *
     * Because the AprilTags are on the bottom of the CELL,
     * seeing the tag tells us which CELL is facing downward.
     */
    private CellSide getDownCell(int tagId) {

        // RED opposite-audience CELL
        if (tagId >= 30 && tagId <= 33) {
            return CellSide.OPPOSITE_AUDIENCE;
        }

        // RED audience CELL
        if (tagId >= 34 && tagId <= 37) {
            return CellSide.AUDIENCE;
        }

        // BLUE audience CELL
        if (tagId >= 38 && tagId <= 41) {
            return CellSide.AUDIENCE;
        }

        // BLUE opposite-audience CELL
        if (tagId >= 42 && tagId <= 45) {
            return CellSide.OPPOSITE_AUDIENCE;
        }

        return CellSide.UNKNOWN;
    }

    /**
     * The opposite CELL from the downward CELL
     * is the upward-facing shooting CELL.
     */
    private CellSide getUpCell(int tagId) {

        CellSide downCell =
                getDownCell(tagId);

        if (downCell == CellSide.AUDIENCE) {
            return CellSide.OPPOSITE_AUDIENCE;
        }

        if (downCell == CellSide.OPPOSITE_AUDIENCE) {
            return CellSide.AUDIENCE;
        }

        return CellSide.UNKNOWN;
    }

    // =========================================================
    // STOP
    // =========================================================

    @Override
    public void stop() {

        if (portal != null) {
            portal.close();
        }
    }
}