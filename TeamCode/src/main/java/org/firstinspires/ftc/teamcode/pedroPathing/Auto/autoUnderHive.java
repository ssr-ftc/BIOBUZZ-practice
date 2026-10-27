package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.PathTimer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.intakeLM0;

@Autonomous(name = "badAuto3", group = "Autonomous")
@Disabled
public class autoUnderHive extends OpMode {
    private Follower follower;
    private PathTimer pathTimer, opModeTimer;
    private org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer timers;
    private intakeLM0 intake;
    private depoLM0 depo;

    public enum PathState {
        SHOOT1,
        STARTING_COORDINATE,
        END_PATH_1,
        END_PATH_2,
        END_PATH_NEW,
        SHOOT2,
        WAIT_AT_PATH2,
        NEARFLOWER,
        INTAKEFLOWER,
        WAITATFLOWER,
        NEAR_OPPFLOWER,
        END_PATH_3,
        DONE
    }

    private PathState pathState;

    private final Pose startingCoordinate = Constants.pose(56, 9, Math.toRadians(-180));
    private final Pose endPath1 = Constants.pose(27, 9, Math.toRadians(-180));
    private final Pose endPath2 = Constants.pose(12.5, 9, Math.toRadians(-180));
    private final Pose endPathnew = new Pose (56, 9, Math.toRadians(90));
    private final Pose approachingFlower = Constants.pose(30,49,Math.toRadians(-180));
    private final Pose flowerNearSlow = new Pose (17,49, Math.toRadians(-180));
    private final Pose oppositFlower = Constants.pose(52, 114.5, Math.toRadians(103));
    private final Pose endPath4 = Constants.pose(11, 109, Math.toRadians(0));

    private Path start_path1, path1_path2, path2_shoot, shoot_approachFlower, apprpoach_nearFlower, nearFlower_oppFlower, shoot2_oppFlower, flower_park;

    public void buildPaths() {
        start_path1 = line(startingCoordinate, endPath1).tangent();
        path1_path2 = line(endPath1, endPath2).tangent();
        path2_shoot = line(endPath2, endPathnew).tangent();
        shoot_approachFlower = line(endPathnew, approachingFlower).linear(endPathnew, approachingFlower);
        apprpoach_nearFlower = line(approachingFlower, flowerNearSlow).tangent();
        shoot2_oppFlower = curve(endPathnew, Constants.pose(60, 69, 0), oppositFlower).tangent();
        nearFlower_oppFlower = curve(flowerNearSlow, Constants.pose(26.5, 100, 0), oppositFlower).tangent();
        flower_park = line(oppositFlower, endPath4).tangent();
    }

    public boolean waitFor(double seconds) {
        return pathTimer.getElapsedTimeSeconds() > seconds;
    }


    public void useDepo() {
        depo.run_using_pid();
        if (depo.reachedvelocity() && !timers.isRunning("depo")) {
            timers.start("depo");
            depo.open_hatch_2();
            intake.turn_on_intake();
        }
        if (timers.checkSecondsLast("depo", 2)) {
            depo.turn_off_deposit();
            depo.close_hatch();
            intake.turn_off_intake();
        }
    }


    public void statePathUpdate() {
        double timeElapsed = pathTimer.getElapsedTimeSeconds();
        switch (pathState) {
            case SHOOT1:
                if (timeElapsed > 1 && !depo.isDepositOn()) {
                    depo.turn_on_deposit(); 
                    setPathState(PathState.STARTING_COORDINATE);
                }
                break;
            case STARTING_COORDINATE:
                if (!depo.isDepositOn()) {
                    //depo.turn_off_deposit();
                    follower.follow(start_path1);
                    setPathState(PathState.END_PATH_1);
                }
                break;
            case END_PATH_1:
                if (!follower.isBusy()) {
                    intake.turn_on_intake();
                    follower.follow(path1_path2);
                    setPathState(PathState.END_PATH_2);
                }
                break;
            case END_PATH_2:
                if (!follower.isBusy()) {
                    //intake.turn_off_intake();
                    //follower.follow(path2_shoot);
                    setPathState(PathState.END_PATH_NEW);
                }
                break;
            case END_PATH_NEW:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.follow(path2_shoot);
                    setPathState(PathState.SHOOT2);
                }
                break;
            case SHOOT2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    depo.turn_on_deposit();
                    setPathState(PathState.NEARFLOWER);
                }
                break;
            case NEARFLOWER:
                if (!follower.isBusy() && !depo.isDepositOn()) {
                    intake.flower_down();
                    follower.follow(shoot_approachFlower);
                    setPathState(PathState.INTAKEFLOWER);
                }
                break;
            case INTAKEFLOWER:
                if (!follower.isBusy()) {
                    intake.turn_on_intake();
                    follower.follow(apprpoach_nearFlower);
                    setPathState(PathState.WAITATFLOWER);
                }
                break;
            case WAITATFLOWER:
                if (!follower.isBusy()){
                    setPathState(PathState.NEAR_OPPFLOWER);
                }
                break;
            case NEAR_OPPFLOWER:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.follow(nearFlower_oppFlower);
                    setPathState(PathState.END_PATH_3);
                }
                break;
                /*
            case WAIT_AT_PATH2:
                if (!follower.isBusy() && !depo.isDepositOn()) {
                    follower.follow(shoot2_oppFlower);
                    setPathState(PathState.END_PATH_3);
                }
                break;
                 */
            case END_PATH_3:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.follow(flower_park);
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

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new PathTimer();
        opModeTimer = new PathTimer();
        follower = Constants.createFollower(hardwareMap);
        //intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake = new intakeLM0(hardwareMap);
        depo = new depoLM0(hardwareMap);
        timers = new org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer();

        buildPaths();
        follower.setPose(startingCoordinate);
        pathState = PathState.SHOOT1;
    }
    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        intake.start();
        depo.start();
        timers.createNew("depo");
    }
    @Override
    public void loop() {
        follower.update();
        useDepo();
        statePathUpdate();
        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }
}
