package org.firstinspires.ftc.teamcode.APAuto.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz.AprilTagReader;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz.BioBuzzVisionConfig;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz.CellOrientation;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz.HiveCell;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz.HivePoseEstimator;
import org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz.TagClusterMapper;

import java.util.Collections;
import java.util.List;
import java.util.Locale;


@Autonomous(name = "AROUNDVisionAuto", group = "Autonomous")
public class AROUNDVisionAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private AprilTagReader aprilTags;
    private List<HiveCell> hiveCells = Collections.emptyList();
    private boolean wasShootable = false;


    public enum PathState {
        STARTING_COORDINATE,
        END_PATH_1,
        END_PATH_2,
        WAIT_AT_PATH2,
        END_PATH_3,
        DONE
    }
    private AROUNDVisionAuto.PathState pathState;

    private final Pose startingCoordinate = new Pose(56, 9, Math.toRadians(-180));
    private final Pose endPath1 = new Pose(27, 9, Math.toRadians(-180));
    private final Pose endPath2 = new Pose(11.5, 9, Math.toRadians(-180));
    private final Pose endPath3 = new Pose(46, 111.5, Math.toRadians(116));
    private final Pose endPath4 = new Pose(11, 109, Math.toRadians(90));


    private PathChain start_path1, path1_path2, path2_path3, path3_path4;

    public void buildPaths() {
        start_path1 = follower.pathBuilder()
                .addPath(new BezierLine(startingCoordinate, endPath1))
                .setTangentHeadingInterpolation()
                .build();
        path1_path2 = follower.pathBuilder()
                .addPath(new BezierLine(endPath1, endPath2))
                .setTangentHeadingInterpolation()
                .build();
        path2_path3 = follower.pathBuilder()
                .addPath(new BezierCurve(endPath2,
                        new Pose(26, 34.5),
                        new Pose(16, 100.5),
                        endPath3))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        HeadingInterpolator.PiecewiseNode.linear(0, 0.292, endPath2.getHeading(), Math.toRadians(90)),
                        new HeadingInterpolator.PiecewiseNode(0.292, 0.755, HeadingInterpolator.tangent),
                        HeadingInterpolator.PiecewiseNode.linear(0.755, 1, Math.toRadians(58), endPath3.getHeading())
                ))
                .build();
        path3_path4 = follower.pathBuilder()
                .addPath(new BezierLine(endPath3, endPath4))
                .setTangentHeadingInterpolation()
                .build();
    }

    public boolean waitFor(double seconds) {
        return pathTimer.getElapsedTimeSeconds() > seconds;
    }

    public void statePathUpdate() {
        double timeElapsed = pathTimer.getElapsedTimeSeconds();

        switch (pathState) {
            case STARTING_COORDINATE:
                if (timeElapsed > 1) {
                    follower.followPath(start_path1, true);
                    setPathState(PathState.END_PATH_1);
                }
                break;
            case END_PATH_1:
                if (!follower.isBusy()) {
                    follower.followPath(path1_path2, 0.4, true);
                    setPathState(PathState.END_PATH_2);
                }
                break;
            case END_PATH_2:
                if (!follower.isBusy()) {
                    follower.followPath(path2_path3, true);
                    setPathState(PathState.WAIT_AT_PATH2);
                }
                break;
            case WAIT_AT_PATH2:
                if (!follower.isBusy()) {
                    setPathState(PathState.END_PATH_3);
                }
                break;
            case END_PATH_3:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(path3_path4, true);
                    setPathState(PathState.DONE);
                }
                break;
            case DONE:
                if (!follower.isBusy()) {
                    telemetry.addLine("finished");
                }
                break;
            default:
                telemetry.addLine("error somewehre in the code");
                break;
        }
    }

    public void setPathState(AROUNDVisionAuto.PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setCentripetalScaling(0.0015);
        follower.getConstants().setBEZIER_CURVE_SEARCH_LIMIT(100);

        buildPaths();
        follower.setPose(startingCoordinate);

        aprilTags = new AprilTagReader();
        aprilTags.init(hardwareMap);

        pathState = PathState.STARTING_COORDINATE;
    }

    @Override
    public void stop() {
        if (aprilTags != null) {
            aprilTags.close();
        }
    }

    private void updateAprilTags() {
        hiveCells = HivePoseEstimator.estimate(
                TagClusterMapper.filterAll(aprilTags.getDetections()));
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        updateAprilTags();
        statePathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Camera", aprilTags.isStreaming() ? "STREAMING" : "NOT STREAMING");

        boolean anyShootable = false;
        if (hiveCells.isEmpty()) {
            telemetry.addLine("AprilTag: none");
        }
        for (HiveCell cell : hiveCells) {
            boolean shootable = isShootable(cell);
            anyShootable = anyShootable || shootable;
            if (!cell.poseValid) {
                telemetry.addLine(String.format(
                        Locale.US,
                        "%s  %s  SHOOTABLE NO  tags %s  no pose",
                        cell.cluster.displayName(),
                        cell.orientation.name(),
                        formatTagIds(cell.visibleTagIds)));
                continue;
            }
            double cameraX = cell.relativeX - BioBuzzVisionConfig.CAMERA_RIGHT_OF_CENTER_INCHES;
            double cameraY = cell.relativeY
                    - BioBuzzVisionConfig.CAMERA_FORWARD_OF_CENTER_INCHES
                    - BioBuzzVisionConfig.OPENING_FORWARD_OF_TAGS_INCHES;
            double cameraZ = cell.relativeZ - BioBuzzVisionConfig.OPENING_ABOVE_TAGS_INCHES;
            telemetry.addLine(String.format(
                    Locale.US,
                    "%s  %s  SHOOTABLE %s  tags %s  camera %.1f in  heading %.1f deg",
                    cell.cluster.displayName(),
                    cell.orientation.name(),
                    shootable ? "YES" : "NO",
                    formatTagIds(cell.visibleTagIds),
                    cameraDistanceInches(cameraX, cameraY, cameraZ),
                    headingFromRobotDegrees(cameraX, cameraY)));
        }
        telemetry.addData("Shootable", anyShootable ? "Yes" : "No");

        if (anyShootable && !wasShootable) {
            gamepad1.rumbleBlips(3);
        }
        wasShootable = anyShootable;

        telemetry.update();
    }

    /**
     * A cell is shootable when its tags show it facing up. Distance and
     * bearing do not block this; the path can still be moving.
     */
    private static boolean isShootable(HiveCell cell) {
        return cell != null
                && cell.detected
                && cell.poseValid
                && cell.poseTagCount >= 1
                && cell.orientation == CellOrientation.FACING_UP
                && cell.meanDecisionMargin >= BioBuzzVisionConfig.MIN_MARGIN_FOR_ORIENTATION;
    }

    /** Straight-line inches from the camera lens to the fused tag point. */
    private static double cameraDistanceInches(double cameraX, double cameraY, double cameraZ) {
        return Math.hypot(Math.hypot(cameraX, cameraY), cameraZ);
    }

    /**
     * Degrees from the direction the robot is facing.
     * Positive is to the robot's right, negative is to the left, and 180 is directly behind.
     */
    private static double headingFromRobotDegrees(double cameraX, double cameraY) {
        double yaw = BioBuzzVisionConfig.CAMERA_YAW_RADIANS;
        double right = cameraX * Math.cos(yaw) - cameraY * Math.sin(yaw);
        double forward = cameraX * Math.sin(yaw) + cameraY * Math.cos(yaw);
        double degrees = Math.toDegrees(Math.atan2(right, forward));
        if (degrees <= -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }

    private static String formatTagIds(int[] ids) {
        if (ids == null || ids.length == 0) {
            return "(none)";
        }
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < ids.length; i++) {
            if (i > 0) {
                builder.append(',');
            }
            builder.append(ids[i]);
        }
        return builder.toString();
    }

}
