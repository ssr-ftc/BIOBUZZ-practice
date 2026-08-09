package org.firstinspires.ftc.teamcode.OFSB2.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
@Autonomous

public class PedroPathingtest extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public enum PathState {
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT

        START_TO_ENDOFPATH1,
        //drive start pos shoot pos
        PATH1_TO_ENDOFPATH2,
        //shoot preload
        PATH2_TO_ENDOFPATH3,
        PATH3_TO_ENDOFPATH4,
        PATH4_TO_ENDOFPATH5,
        DONE

    }
    PathState pathState;

    private final Pose starting = new Pose(70.85831960461286, 134.72981878088962, Math.toRadians(90));
    private final Pose path1complete = new Pose(47.40449664771886, 70.25789384267753, Math.toRadians((90)));
    private final Pose path2complete = new Pose(93.91480375429161, 40.852017956911546, Math.toRadians(360));
    private final Pose path3complete = new Pose(23.31136738056013, 47.088962108731465, Math.toRadians(55));
    private final Pose path4complete = new Pose(71.20195460911026, 99.05104624197372, Math.toRadians(55));
    private final Pose path5complete = new Pose(70.85831960461286, 129.72981878088962, Math.toRadians(90));


    private PathChain path1_path2, path2_path3, path3_path4, path4_path5, path5_path6;
    public void buildPaths() {
        path1_path2 = follower.pathBuilder()
                .addPath(new BezierLine(starting, path1complete))
                .setLinearHeadingInterpolation(starting.getHeading(), path1complete.getHeading())
                .build();
        path2_path3 = follower.pathBuilder()
                .addPath(new BezierLine(path1complete, path2complete))
                .setLinearHeadingInterpolation(path1complete.getHeading(), path2complete.getHeading())
                .build();
        path3_path4 = follower.pathBuilder()
                .addPath(new BezierLine(path2complete, path3complete))
                .setLinearHeadingInterpolation(path2complete.getHeading(), path3complete.getHeading())
                .build();
        path4_path5 = follower.pathBuilder()
                .addPath(new BezierLine(path3complete, path4complete))
                .setLinearHeadingInterpolation(path3complete.getHeading(), path4complete.getHeading())
                .build();
        path5_path6 = follower.pathBuilder()
                .addPath(new BezierLine(path4complete, path5complete))
                .setLinearHeadingInterpolation(path4complete.getHeading(), path5complete.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch (pathState) {
            case START_TO_ENDOFPATH1:
                follower.followPath(path1_path2, true);
                setPathState(PathState.PATH1_TO_ENDOFPATH2);
                break;
            case PATH1_TO_ENDOFPATH2:
                if(!follower.isBusy()) {
                    follower.followPath(path2_path3, true);
                    setPathState(PathState.PATH2_TO_ENDOFPATH3);
                }
                break;
            case PATH2_TO_ENDOFPATH3:
                if(!follower.isBusy()) {
                    follower.followPath(path3_path4,  true);
                    setPathState(PathState.PATH3_TO_ENDOFPATH4);
                }
                break;
            case PATH3_TO_ENDOFPATH4:
                if(!follower.isBusy()) {
                    follower.followPath(path4_path5, true);
                    setPathState(PathState.PATH4_TO_ENDOFPATH5);
                }
                break;
            case PATH4_TO_ENDOFPATH5:
                if(!follower.isBusy()) {
                    follower.followPath(path5_path6, true);
                    setPathState(PathState.DONE);
                }
            case DONE:
                if(!follower.isBusy()){
                    telemetry.addLine("all patsh completse");
                }
                break;
            default:
                telemetry.addLine("not working pls fix");
                break;
        }

    }
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathState = PathState.START_TO_ENDOFPATH1;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(starting);
    }
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());


    }
}

