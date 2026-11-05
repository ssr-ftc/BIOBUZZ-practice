package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Robot0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.intakeLM0;

@Autonomous(name = "newHalfAuto", group = "Autonomous")
public class newHalfAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer timers;
    private intakeLM0 intake;
    private depoLM0 depo;

    public enum PathState {
        START_SLOWPOS,
        SLOWPOS_FINISHINTAKE,
        FINISHINTAKE_SHOOT2,
        SHOOT2_NEARINGFLOWER,
        NEARINGFLOWER_INTAKEFLOWER,
        INTAKEFLOWER_BACKUP,
        BACKUP_PARK,
        DONE

    }

    private PathState pathState;

    private final Pose startingCoordinate = new Pose(57, 8, Math.toRadians(180));
    private final Pose slowpos = new Pose(18, 8, Math.toRadians(180));
    private final Pose finishIntaking = new Pose(12.5, 9, Math.toRadians(180));
    private final Pose nearingFlower = new Pose(24,49, Math.toRadians(180));
    private final Pose intakeFlower = new Pose(14,47,Math.toRadians(180));
    private final Pose park = new Pose (13,95,Math.toRadians(180));


    private Path start_mid, mid_fullIntake, fullIntake_shoot2, shoot2_nearingflower,
            nearingflower_intakeflower, intake_backup, backupflower_park;

    public void buildPaths() {
        start_mid = line(startingCoordinate, slowpos).tangent();
        mid_fullIntake = line(slowpos, finishIntaking).linear(startingCoordinate.heading(), startingCoordinate.heading());
        fullIntake_shoot2 = line(finishIntaking, startingCoordinate).reverseTangent();
        shoot2_nearingflower = line(startingCoordinate, nearingFlower).tangent();
        nearingflower_intakeflower = line(nearingFlower, intakeFlower).tangent();
        intake_backup = line(intakeFlower, nearingFlower).reverseTangent();
        backupflower_park = line(nearingFlower, park).tangent();
        /*
        path2_shoot = line(endPath2, endPathnew).reverseTangent();
        shoot_approachFlower = line(endPathnew, approachingFlower).linear(endPathnew.heading(), approachingFlower.heading());
        apprpoach_nearFlower = line(approachingFlower, flowerNearSlow).tangent();
        nearFlower_oppFlower = curve(flowerNearSlow, new Pose(26.5, 100), oppositFlower).tangent();
        flower_park = line(oppositFlower, endPath4).tangent();

         */
    }

    public boolean waitFor(double seconds) {
        return pathTimer.seconds() > seconds;
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
        double timeElapsed = pathTimer.seconds();
        switch (pathState) {
            case START_SLOWPOS:
                if (!follower.isBusy()) {
                    follower.follow(start_mid);
                    setPathState(PathState.SLOWPOS_FINISHINTAKE);
                }
                break;
            case SLOWPOS_FINISHINTAKE:
                if (!follower.isBusy()) {
                    Robot0.setPathPower(0.4);
                    follower.follow(mid_fullIntake);
                    setPathState(PathState.FINISHINTAKE_SHOOT2);
                }
                break;
            case FINISHINTAKE_SHOOT2:
                if (!follower.isBusy()) {
                    Robot0.setPathPower(1.0);
                    follower.follow(fullIntake_shoot2);
                    setPathState(PathState.SHOOT2_NEARINGFLOWER);
                }
                break;
            case SHOOT2_NEARINGFLOWER:
                if (!follower.isBusy()) {
                    follower.follow(shoot2_nearingflower);
                    setPathState(PathState.NEARINGFLOWER_INTAKEFLOWER );
                }
                break;
            case NEARINGFLOWER_INTAKEFLOWER:
                if (!follower.isBusy()) {
                    Robot0.setPathPower(0.4);
                    follower.follow(nearingflower_intakeflower);
                    setPathState(PathState.INTAKEFLOWER_BACKUP);
                }
                break;
            case INTAKEFLOWER_BACKUP:
                if (!follower.isBusy()) {
                    Robot0.setPathPower(1.0);
                    follower.follow(intake_backup);
                    setPathState(PathState.BACKUP_PARK);
                }
                break;
            case BACKUP_PARK:
                if (!follower.isBusy()) {
                    follower.follow(backupflower_park);
                    setPathState(PathState.DONE);
                }
                break;
            case DONE:
                if (!follower.isBusy()) {
                    telemetry.addLine("its workokrrkinngg!!");
                }
                break;
            default:
                telemetry.addLine("error somewehre in the code");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.reset();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Robot0.createFollower(hardwareMap);
        intake = new intakeLM0(hardwareMap);
        depo = new depoLM0(hardwareMap);
        timers = new org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer();

        buildPaths();
        follower.setPose(startingCoordinate);
        pathState = PathState.START_SLOWPOS;
    }

    @Override
    public void start() {
        opModeTimer.reset();
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
