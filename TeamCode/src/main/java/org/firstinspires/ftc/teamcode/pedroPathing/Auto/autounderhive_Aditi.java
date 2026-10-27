package org.firstinspires.ftc.teamcode.pedroPathing.Auto;
//Needs near fine tuning version 0.1.3.0
import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.DriveLimits;
import org.firstinspires.ftc.teamcode.pedroPathing.PathTimer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.intakeLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.turret;

@Autonomous(name = "badAuto2", group = "Autonomous")
@Disabled
public class autounderhive_Aditi extends OpMode {
    private Follower follower;
    private PathTimer pathTimer, opModeTimer, stallTimer;
    private org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer timers;
    private intakeLM0 intake;
    private depoLM0 depo;
    private turret turret;

    public enum PathState {
        SHOOT1,
        PATH1,
        PATH2,
        PATH3,
        SHOOT2,
        PATH4,
        PATH4_INTAKE,
        PATH5,
        PATH6,
        DRIVE_SHOOT2,
        SHOOT3,
        PATH8,
        INTAKE_FLOWER,
        SHOOT4,
        PATH7,
        PARK,
        DONE
    }

    private PathState pathState;
    private boolean gardenHoldStarted = false;
    private boolean path4Started = false;
    private boolean path4FlowerDown = false;
    private boolean flower2Approach = false;
    private boolean flower2ClearStarted = false;
    private boolean flowerPush = false;
    private double turretTargetDeg = 90;
    /** Heading 180 with the turret at 90 points at the first hive. Turret = robot heading - 90. */
    private static final double TURRET_FIELD_AIM_DEG = 90;
    /** Far shot at the second hive. */
    private static final double SECOND_HIVE_TURRET_DEG = -45;
    /** A shot counts as aimed once the turret is within this many degrees of the target. */
    private static final double TURRET_TOLERANCE_DEG = 7.0;
    /** Flower starts down this long before the path 4 stop, based on current speed. */
    private static final double FLOWER_LEAD_SECONDS = 1.5;
    /** Stopped intake at the path 4 flower and at the far flower. */
    private static final double FLOWER_INTAKE_SECONDS = 2.0;
    /** Stay down after reaching the far flower so the flower piece can clear the flowers. */
    private static final double FLOWER2_CLEAR_SECONDS = 2.0;
    /**
     * Last part of the drive into the far flower. 36 inches at this power is a little
     * over 1.5 seconds and slow enough that the flower piece does not slam in.
     */
    private static final double FAR_FLOWER_APPROACH_INCHES = 36;
    private static final double FAR_FLOWER_APPROACH_POWER = 0.55;
    /** Straight garden intake. The curved visualizer lines are not used. Intake stays on for one second after the robot stops. */
    private static final double GARDEN_INTAKE_POWER = 0.4;
    private static final double GARDEN_HOLD_SECONDS = 1.0;
    /** Last 40% of the drive from the second shot to the near flower. */
    private static final double PATH4_SLOW_START = 0.6;
    private static final double PATH4_SLOW_POWER = 0.2;
    /**
     * The slow flower approach can stall a few inches short. After a short stall the
     * robot gets this much power to finish the drive onto the flower pose.
     */
    private static final double FLOWER_PUSH_POWER = 0.5;
    /** Slower than this, in inches per second, counts as stalled. */
    private static final double STALL_SPEED = 1.5;
    private static final double STALL_PUSH_SECONDS = 0.25;
    /** Still stalled this long while pushing means the robot is pressed against the flower. */
    private static final double STALL_AT_FLOWER_SECONDS = 0.75;
    /** Share of the drive from the second shot to flower2 used to turn to 90 degrees. */
    private static final double FLOWER2_TURN_END_T = 0.3;

    // Visualizer poses are degrees. Pedro Pathing 2 stores heading in radians.
    private final Pose start = Constants.pose(56, 9, Math.toRadians(180));
    private final Pose straightGardenIntakeEnd = Constants.pose(27, 9, Math.toRadians(180));
    private final Pose straightGardenIntake2End = Constants.pose(12.5, 9, Math.toRadians(180));
    private final Pose shootaftergarden = Constants.pose(56, 9, Math.toRadians(150));
    private final Pose flower1 = Constants.pose(13, 53, Math.toRadians(180));
    private final Pose exitflower1 = Constants.pose(21.4, 53, Math.toRadians(180));
    private final Pose shootflower1 = Constants.pose(22.0147, 88.2872, Math.toRadians(180));
    private final Pose shoot2End = Constants.pose(47.0839, 135.1803, Math.toRadians(-118.1291));
    private final Pose flower2 = Constants.pose(49.7998, 120.8323, Math.toRadians(90));
    private final Pose getready4flower2 = Constants.pose(45.5631, 124.8262, Math.toRadians(90));
    private final Pose parkEnd = Constants.pose(2.2065, 113.9036, Math.toRadians(0));

    private Path straightGardenIntake, straightGardenIntake2, path3, path4, path5, path6, shoot2, path8, path7, park;

    public void buildPaths() {
        straightGardenIntake = line(start, straightGardenIntakeEnd).tangent();
        straightGardenIntake2 = line(straightGardenIntakeEnd, straightGardenIntake2End).tangent();
        path3 = line(straightGardenIntake2End, shootaftergarden).linear(straightGardenIntake2End, shootaftergarden);
        path4 = line(shootaftergarden, flower1).linear(shootaftergarden, flower1);
        path5 = line(flower1, exitflower1).tangent();
        path6 = line(exitflower1, shootflower1).constant(shootflower1);
        shoot2 = line(shootflower1, shoot2End).tangent();
        path8 = line(shoot2End, flower2).linear(shoot2End, flower2);
        path7 = line(flower2, getready4flower2).constant(getready4flower2);
        park = line(getready4flower2, parkEnd).linear(getready4flower2, parkEnd);
    }

    /**
     * The hatch stays closed during intake. It only opens once a shot has been requested
     * (deposit on) and the flywheel has reached the right velocity.
     */
    public void useDepo() {
        depo.run_using_pid();
        if (depo.isDepositOn() && depo.reachedvelocity() && !timers.isRunning("depo")) {
            timers.start("depo");
            depo.open_hatch(turret.getPositionDeg());
            intake.turn_on_intake();
        }
        if (timers.checkSecondsLast("depo", 2)) {
            depo.turn_off_deposit();
            depo.close_hatch();
            intake.turn_off_intake();
        }
    }

    public void statePathUpdate() {
        switch (pathState) {
            case SHOOT1:
                if (pathTimer.getElapsedTimeSeconds() > 1 && !depo.isDepositOn() && turretAimed()) {
                    depo.turn_on_deposit();
                    setPathState(PathState.PATH1);
                }
                break;
            case PATH1:
                if (!depo.isDepositOn()) {
                    follower.follow(straightGardenIntake);
                    setPathState(PathState.PATH2);
                }
                break;
            case PATH2:
                if (!follower.isBusy()) {
                    intake.turn_on_intake();
                    //DriveLimits.setMaxPower(follower, GARDEN_INTAKE_POWER);
                    follower.follow(straightGardenIntake2);
                    setPathState(PathState.PATH3);
                }
                break;
            case PATH3:
                if (follower.isBusy()) {
                    break;
                }
                if (!gardenHoldStarted) {
                    pathTimer.resetTimer();
                    gardenHoldStarted = true;
                    break;
                }
                if (pathTimer.getElapsedTimeSeconds() > GARDEN_HOLD_SECONDS) {
                    intake.turn_off_intake();
                    DriveLimits.setMaxPower(follower, 1.0);
                    aimTurret(150);
                    follower.follow(path3);
                    setPathState(PathState.SHOOT2);
                }
                break;
            case SHOOT2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1 && turretAimed()) {
                    depo.turn_on_deposit();
                    setPathState(PathState.PATH4);
                }
                break;
            case PATH4:
                if (!path4Started) {
                    if (!follower.isBusy() && !depo.isDepositOn()) {
                        DriveLimits.setMaxPower(follower, 1.0);
                        intake.turn_off_intake();
                        intake.flower_up();
                        follower.follow(path4);
                        path4Started = true;
                    }
                    break;
                }
                boolean path4Slow = follower.completion() >= PATH4_SLOW_START;
                if (path4Slow && !flowerPush) {
                    DriveLimits.setMaxPower(follower, PATH4_SLOW_POWER);
                }
                if (!path4FlowerDown && leadTimeReached(exitflower1, FLOWER_LEAD_SECONDS)) {
                    intake.flower_down();
                    path4FlowerDown = true;
                }
                if (reachedFlower(path4Slow)) {
                    intake.flower_down();
                    intake.turn_on_intake();
                    setPathState(PathState.PATH4_INTAKE);
                }
                break;
            case PATH4_INTAKE:
                if (pathTimer.getElapsedTimeSeconds() > FLOWER_INTAKE_SECONDS) {
                    intake.turn_off_intake();
                    intake.flower_up();
                    DriveLimits.setMaxPower(follower, 1.0);
                    follower.follow(path5);
                    setPathState(PathState.PATH5);
                }
                break;
            case PATH5:
                if (!follower.isBusy()) {
                    DriveLimits.setMaxPower(follower, 1.0);
                    flowerPush = false;
                    setTurretDeg(SECOND_HIVE_TURRET_DEG);
                    follower.follow(path6);
                    setPathState(PathState.PATH6);
                }
                break;
            case PATH6:
                if (!follower.isBusy()) {
                    DriveLimits.setMaxPower(follower, 1.0);
                    follower.follow(shoot2);
                    setPathState(PathState.DRIVE_SHOOT2);
                }
                break;
            case DRIVE_SHOOT2:
                if (!follower.isBusy() && turretAimed()) {
                    intake.turn_off_intake();
                    intake.flower_up();
                    depo.turn_on_deposit();
                    setPathState(PathState.SHOOT3);
                }
                break;
            case SHOOT3:
                if (depo.isDepositOn()) {
                    break;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    DriveLimits.setMaxPower(follower, 1.0);
                    flowerPush = false;
                    intake.flower_up();
                    follower.follow(path8);
                    setPathState(PathState.PATH8);
                }
                break;
            case PATH8:
                if (!follower.isBusy()) {
                    DriveLimits.setMaxPower(follower, 1.0);
                    flowerPush = false;
                    follower.follow(path7);
                    setPathState(PathState.PATH7);
                }
                break;
            case INTAKE_FLOWER:
                if (pathTimer.getElapsedTimeSeconds() > FLOWER_INTAKE_SECONDS) {
                    intake.flower_up();
                    depo.turn_on_deposit();
                    setPathState(PathState.SHOOT4);
                }
                break;
            case SHOOT4:
                if (!depo.isDepositOn() && pathTimer.getElapsedTimeSeconds() > 1) {
                    intake.flower_up();
                    intake.turn_off_intake();
                    DriveLimits.setMaxPower(follower, 1.0);
                    flowerPush = false;
                    follower.follow(park);
                    setPathState(PathState.PARK);
                }
                break;
            case PATH7:
                if (!flower2Approach && distanceTo(getready4flower2) <= FAR_FLOWER_APPROACH_INCHES) {
                    flower2Approach = true;
                    intake.flower_down();
                    DriveLimits.setMaxPower(follower, FAR_FLOWER_APPROACH_POWER);
                }
                if (!reachedFlower(flower2Approach)) {
                    break;
                }
                if (!flower2ClearStarted) {
                    flower2ClearStarted = true;
                    pathTimer.resetTimer();
                    DriveLimits.setMaxPower(follower, 1.0);
                    intake.flower_down();
                    break;
                }
                if (pathTimer.getElapsedTimeSeconds() < FLOWER2_CLEAR_SECONDS) {
                    intake.flower_down();
                    break;
                }
                intake.flower_down();
                intake.turn_on_intake();
                setPathState(PathState.INTAKE_FLOWER);
                break;
            case PARK:
                if (pathDone(parkEnd)) {
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

    /** Keeps the shooter on the first hive. Heading 180 uses turret 90. */
    private void aimTurret(double robotHeadingDeg) {
        setTurretDeg(robotHeadingDeg - TURRET_FIELD_AIM_DEG);
    }

    private void setTurretDeg(double degrees) {
        turretTargetDeg = degrees;
        turret.setPositionDeg(degrees);
    }

    private boolean turretAimed() {
        return Math.abs(turret.getPositionDeg() - turretTargetDeg) <= TURRET_TOLERANCE_DEG;
    }

    /**
     * Pedro only ends a path at t >= 0.99 and ignores a stall past t = 0.8, so a slow
     * approach that stalls a few inches short of the flower never finishes. During the
     * slow approach, a stall gets more power so Pedro can finish on the flower pose.
     * If the robot still cannot move while pushing, it is pressed against the flower.
     */
    private boolean reachedFlower(boolean slowApproach) {
        if (!follower.isBusy()) {
            return true;
        }
        if (!slowApproach || Math.hypot(follower.velocity().vx, follower.velocity().vy) > STALL_SPEED) {
            stallTimer.resetTimer();
            return false;
        }
        if (!flowerPush) {
            if (stallTimer.getElapsedTimeSeconds() > STALL_PUSH_SECONDS) {
                flowerPush = true;
                DriveLimits.setMaxPower(follower, FLOWER_PUSH_POWER);
                stallTimer.resetTimer();
            }
            return false;
        }
        if (stallTimer.getElapsedTimeSeconds() > STALL_AT_FLOWER_SECONDS) {
            follower.stop();
            return true;
        }
        return false;
    }

    private boolean pathDone(Pose target) {
        if (!follower.isBusy()) {
            return true;
        }
        double speed = Math.hypot(follower.velocity().vx, follower.velocity().vy);
        if (follower.completion() >= 0.8 && speed < 2.0 && distanceTo(target) <= 8) {
            follower.stop();
            return true;
        }
        return false;
    }

    private double distanceTo(Pose target) {
        Pose pose = follower.pose();
        double dx = target.x() - pose.x();
        double dy = target.y() - pose.y();
        return Math.hypot(dx, dy);
    }

    /** True once the robot is about {@code seconds} away from the pose at its current speed. */
    private boolean leadTimeReached(Pose target, double seconds) {
        double speed = Math.hypot(follower.velocity().vx, follower.velocity().vy);
        if (speed < 8) {
            return distanceTo(target) < 12;
        }
        return distanceTo(target) <= speed * seconds;
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new PathTimer();
        opModeTimer = new PathTimer();
        stallTimer = new PathTimer();
        follower = Constants.createFollower(hardwareMap);
        intake = new intakeLM0(hardwareMap);
        intake.flower_up();
        depo = new depoLM0(hardwareMap);
        depo.close_hatch(); // hatch starts closed for intake
        turret = new turret(hardwareMap);
        timers = new org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer();

        buildPaths();
        follower.setPose(start);
        pathState = PathState.SHOOT1;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        intake.start();
        depo.start();
        depo.close_hatch(); // in case depo.start() resets the servo
        turret.start();
        turret.ResetTurretEncoders();
        aimTurret(180);
        timers.createNew("depo");
    }

    @Override
    public void loop() {
        follower.update();
        turret.run_using_pid();
        useDepo();
        statePathUpdate();
        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Turret target", turretTargetDeg);
        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.update();
    }
}