package org.firstinspires.ftc.teamcode.OFSB2.Auto.TestCurveTangent;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;


@Autonomous(name = "decodeAuto", group = "Autonomous")
public class decodeAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;


    public enum PathState {
        STARTING_COORDINATE,
        PATH1_COMPLETE,
        PATH2_COMPLETE,
        PATH3_COMPLETE,
        PATH4_COMPLETE,
        PATH5_COMPLETE,
        PATH6_COMPLETE,
        DONE
    }

    private decodeAuto.PathState pathState;


    private final Pose startingCoordinate = new Pose(32, 134, Math.toRadians(90));
    private final Pose endPath1 = new Pose(43, 124, Math.toRadians(140));
    private final Pose endPath2 = new Pose(25, 103, Math.toRadians(180));
    private final Pose endPath3 = new Pose(43, 124, Math.toRadians(140));
    private final Pose endPath4 = new Pose(25, 80, Math.toRadians(180));
    private final Pose endPath5 = new Pose(43, 124, Math.toRadians(140));
    private final Pose endPath6 = new Pose(25, 55, Math.toRadians(180));
    private final Pose endingCoordinate = new Pose(43, 124, Math.toRadians(140));


    private PathChain start_path1, path1_path2, path2_path3, path3_path4, path4_path5, path5_path6, path6_finish;

    public void buildPaths() {
        //com.pedropathing.paths.PathConstraints precision = new com.pedropathing.paths.PathConstraints(0.995, 0.1, 0.1, 0.007, 100, 1.2, 100, 1);
        start_path1 = follower.pathBuilder()
                .addPath(new BezierLine(startingCoordinate, endPath1))
                .setLinearHeadingInterpolation(startingCoordinate.getHeading(), endPath1.getHeading())
                .build();

        path1_path2 = follower.pathBuilder()
                .addPath(new BezierCurve(endPath1,
                        new Pose(45, 117),
                        new Pose(52, 109),
                        new Pose(48, 98),
                        endPath2))
                .setLinearHeadingInterpolation(endPath1.getHeading(), endPath2.getHeading())
                .build();

        path2_path3 = follower.pathBuilder()
                .addPath(new BezierLine(endPath2, endPath3))
                .setLinearHeadingInterpolation(endPath2.getHeading(), endPath3.getHeading())
                .build();

        path3_path4 = follower.pathBuilder()
                .addPath(new BezierCurve(endPath3,
                        new Pose(55, 93),
                        new Pose(52, 77),
                        new Pose(45, 77),
                        endPath4))
                .setLinearHeadingInterpolation(endPath3.getHeading(), endPath4.getHeading())
                .build();

        path4_path5 = follower.pathBuilder()
                .addPath(new BezierLine(endPath4, endPath5))
                .setLinearHeadingInterpolation(endPath4.getHeading(), endPath5.getHeading())
                .build();
        path5_path6 = follower.pathBuilder()
                .addPath(new BezierCurve(endPath5,
                        new Pose(62, 64),
                        new Pose(68, 58),
                        new Pose(46, 54),
                        endPath6))
                .setLinearHeadingInterpolation(endPath5.getHeading(), endPath6.getHeading())
                .build();
        path6_finish = follower.pathBuilder()
                .addPath(new BezierLine(endPath6, endingCoordinate))
                .setLinearHeadingInterpolation(endPath6.getHeading(), endingCoordinate.getHeading())
                .build();


    }

    public boolean waitFor(double seconds) {
        return pathTimer.getElapsedTimeSeconds() > seconds;
    }
    public void statePathUpdate() {
        double timeElapsed = pathTimer.getElapsedTimeSeconds();

        switch (pathState) {
            case STARTING_COORDINATE:
                follower.followPath(start_path1, true);
                setPathState(decodeAuto.PathState.PATH1_COMPLETE);
                break;

            case PATH1_COMPLETE:
                if (!follower.isBusy() && timeElapsed > 2.5 ) {
                    follower.followPath(path1_path2, true);
                    setPathState(decodeAuto.PathState.PATH2_COMPLETE);
                }
                break;

            case PATH2_COMPLETE:
                if (!follower.isBusy() ) {
                    follower.followPath(path2_path3, true);
                    setPathState(PathState.PATH3_COMPLETE);
                }
                break;

            case PATH3_COMPLETE:
                if (!follower.isBusy() && timeElapsed > 2.5) {
                    follower.followPath(path3_path4, true);
                    setPathState(PathState.PATH4_COMPLETE);
                }
                break;

            case PATH4_COMPLETE:
                if (!follower.isBusy()) {
                    follower.followPath(path4_path5, true);
                    setPathState(PathState.PATH5_COMPLETE);
                }
                break;
            case PATH5_COMPLETE:
                if (!follower.isBusy() && timeElapsed > 2.5) {
                    follower.followPath(path5_path6, true);
                    setPathState(PathState.PATH6_COMPLETE);
                }
                break;
            case PATH6_COMPLETE:
                if (!follower.isBusy()) {
                    follower.followPath(path6_finish, true);
                    setPathState(PathState.DONE);
                }
                break;
            case DONE:
                if(!follower.isBusy()) {
                    telemetry.addLine("finished");
                    break;
                }
            default:
                telemetry.addLine("error somewehre in the code");
                break;
        }
    }





    public void setPathState(decodeAuto.PathState newState) {
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

        pathState = PathState.STARTING_COORDINATE;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

}

