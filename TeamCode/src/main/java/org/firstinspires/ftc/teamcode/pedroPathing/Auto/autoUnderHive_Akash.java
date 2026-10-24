package org.firstinspires.ftc.teamcode.pedroPathing.Auto;
//Version 1.3.2
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.intakeLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.turret;

@Autonomous(name = "UnderHive Akash", group = "Autonomous")
public class autoUnderHive_Akash extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, stallTimer;
    private org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer timers;
    private intakeLM0 intake;
    private depoLM0 depo; 
    private turret turret;

    public enum PathState {
        SHOOT_PRELOAD,
        DRIVE_GARDEN,
        DRIVE_GARDEN_SLOW,
        HOLD_GARDEN,
        SHOOT_NEAR,
        DRIVE_FLOWER1,
        HOLD_FLOWER1,
        BACKUP_FLOWER1,
        DRIVE_FAR_SIDE,
        DRIVE_FAR_SHOOT,
        SHOOT_FAR,
        DROP_FOR_FLOWER2,
        DRIVE_FLOWER2,
        BACK_TO_TURNAROUND,
        BACK_TO_FAR_SHOOT,
        SHOOT_RETURN,
        PARK,
        DONE
    }

    private PathState pathState;
    private boolean gardenHoldStarted = false;
    private boolean flower1DriveStarted = false;
    private boolean flower1Down = false;
    private boolean flower2Slowing = false;
    private boolean flower2HoldStarted = false;
    private boolean flower2DropStarted = false;
    private boolean flowerPush = false;
    private double turretTargetDeg = 90;
    /** Heading 180 with the turret at 90 points at the first hive. Turret = robot heading - 90. */
    private static final double TURRET_FIELD_AIM_DEG = 90;
    /** Far shot at the second hive. Set again on the return so the turret does not stay wherever it drifted. */
    private static final double SECOND_HIVE_TURRET_DEG = -40;
    /** A shot counts as aimed once the turret is within this many degrees of the target. */
    private static final double TURRET_TOLERANCE_DEG = 7.0;
    /** Flower starts down this long before the near flower, based on current speed. */
    private static final double FLOWER_LEAD_SECONDS = 1.5;
    /** Stopped intake at each flower. The intake is already running on the way in. */
    private static final double FLOWER_HOLD_SECONDS = 0.5;
    /** Time for the flower piece to finish dropping at the turnaround before driving in. */
    private static final double FLOWER2_DROP_SECONDS = 0.6;
    /**
     * Last part of the drive into the far flower. 36 inches at this power is a little
     * over 1.5 seconds and slow enough that the flower piece does not slam in.
     */
    private static final double FAR_FLOWER_APPROACH_INCHES = 36;
    private static final double FAR_FLOWER_APPROACH_POWER = 0.4;
    /** Straight garden intake. The curved visualizer lines are not used. Intake stays on for one second after the robot stops. */
    private static final double GARDEN_INTAKE_POWER = 0.4;
    private static final double GARDEN_HOLD_SECONDS = 1.0;
    /** Last 40% of the drive from the near shot to the first flower. */
    private static final double FLOWER1_SLOW_START = 0.6;
    private static final double FLOWER1_SLOW_POWER = 0.2;
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

    // Visualizer poses are degrees. Pedro Pathing 2 stores heading in radians.
    private final Pose start = new Pose(56, 9, Math.toRadians(180));
    private final Pose gardenIntake = new Pose(27, 9, Math.toRadians(180));
    private final Pose gardenIntakeEnd = new Pose(12.5, 9, Math.toRadians(180));
    private final Pose nearShoot = new Pose(56, 9, Math.toRadians(150));
    private final Pose flower1 = new Pose(13, 53, Math.toRadians(180));
    private final Pose flower1Backup = new Pose(21.4, 53, Math.toRadians(180));
    private final Pose farSide = new Pose(22.0147, 88.2872, Math.toRadians(180));
    private final Pose farShoot = new Pose(31.9549, 134.2904, Math.toRadians(-102.1929));
    private final Pose flower2Turnaround = new Pose(46.2013, 108.9895, Math.toRadians(88.0699));
    private final Pose flower2 = new Pose(47.0463, 133.6, Math.toRadians(90));
    private final Pose parkSpot = new Pose(2.2065, 113.9036, Math.toRadians(0));

    private PathChain toGardenIntake, toGardenIntakeEnd, toNearShoot, toFlower1, backupFlower1, toFarSide, toFarShoot, toFlower2Turnaround, toFlower2, backToTurnaround, backToFarShoot, toPark;

    public void buildPaths() {
        toGardenIntake = follower.pathBuilder()
                .addPath(new BezierLine(start, gardenIntake))
                .setTangentHeadingInterpolation()
                .build();

        toGardenIntakeEnd = follower.pathBuilder()
                .addPath(new BezierLine(gardenIntake, gardenIntakeEnd))
                .setTangentHeadingInterpolation()
                .build();

        toNearShoot = follower.pathBuilder()
                .addPath(new BezierLine(gardenIntakeEnd, nearShoot))
                .setLinearHeadingInterpolation(gardenIntakeEnd.getHeading(), nearShoot.getHeading())
                .build();

        toFlower1 = follower.pathBuilder()
                .addPath(new BezierLine(nearShoot, flower1))
                .setLinearHeadingInterpolation(nearShoot.getHeading(), flower1.getHeading())
                .build();

        backupFlower1 = follower.pathBuilder()
                .addPath(new BezierLine(flower1, flower1Backup))
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();

        toFarSide = follower.pathBuilder()
                .addPath(new BezierLine(flower1Backup, farSide))
                .setConstantHeadingInterpolation(farSide.getHeading())
                .build();

        toFarShoot = follower.pathBuilder()
                .addPath(new BezierLine(farSide, farShoot))
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();

        toFlower2Turnaround = follower.pathBuilder()
                .addPath(new BezierLine(farShoot, flower2Turnaround))
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();

        toFlower2 = follower.pathBuilder()
                .addPath(new BezierLine(flower2Turnaround, flower2))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        backToTurnaround = follower.pathBuilder()
                .addPath(new BezierLine(flower2, flower2Turnaround))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        backToFarShoot = follower.pathBuilder()
                .addPath(new BezierLine(flower2Turnaround, farShoot))
                .setLinearHeadingInterpolation(flower2Turnaround.getHeading(), farShoot.getHeading() + 2 * Math.PI)
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(farShoot, parkSpot))
                .setLinearHeadingInterpolation(farShoot.getHeading(), parkSpot.getHeading())
                .build();
    }

    public void useDepo() {
        depo.run_using_pid();
        if (depo.reachedvelocity() && !timers.isRunning("depo")) {
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

    /** Starts the flywheel. The hatch still opens from useDepo once the shooter is up to speed. */
    private void shoot() {
        depo.turn_on_deposit();
    }

    /** Far shots raise the flower and stop the intake before the flywheel starts. */
    private void shootWithFlowerUp() {
        intake.turn_off_intake();
        intake.flower_up();
        shoot();
    }

    /** True once the shot has finished and the robot has waited one second. */
    private boolean shotDone() {
        return !depo.isDepositOn() && pathTimer.getElapsedTimeSeconds() > 1;
    }

    public void statePathUpdate() {
        switch (pathState) {
            case SHOOT_PRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > 1 && !depo.isDepositOn() && turretAimed()) {
                    shoot();
                    setPathState(PathState.DRIVE_GARDEN);
                }
                break;
            case DRIVE_GARDEN:
                if (!depo.isDepositOn()) {
                    intake.turn_on_intake();
                    follower.followPath(toGardenIntake, true);
                    setPathState(PathState.DRIVE_GARDEN_SLOW);
                }
                break;
            case DRIVE_GARDEN_SLOW:
                if (!follower.isBusy()) {
                    intake.turn_on_intake();
                    //follower.setMaxPower(GARDEN_INTAKE_POWER);
                    follower.followPath(toGardenIntakeEnd, 0.4, true);
                    setPathState(PathState.HOLD_GARDEN);
                }
                break;
            case HOLD_GARDEN:
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
                    follower.setMaxPower(1.0);
                    aimTurret(150);
                    follower.followPath(toNearShoot, true);
                    setPathState(PathState.SHOOT_NEAR);
                }
                break;
            case SHOOT_NEAR:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1 && turretAimed()) {
                    shoot();
                    setPathState(PathState.DRIVE_FLOWER1);
                }
                break;
            case DRIVE_FLOWER1:
                if (!flower1DriveStarted) {
                    if (!follower.isBusy() && !depo.isDepositOn()) {
                        follower.setMaxPower(1.0);
                        intake.turn_off_intake();
                        intake.flower_up();
                        follower.followPath(toFlower1, true);
                        flower1DriveStarted = true;
                    }
                    break;
                }
                boolean flower1Slow = follower.getCurrentTValue() >= FLOWER1_SLOW_START;
                if (flower1Slow && !flowerPush) {
                    follower.setMaxPower(FLOWER1_SLOW_POWER);
                }
                if (!flower1Down && (flower1Slow || leadTimeReached(flower1, FLOWER_LEAD_SECONDS))) {
                    intake.flower_down();
                    intake.turn_on_intake();
                    flower1Down = true;
                }
                if (reachedFlower(flower1Slow)) {
                    intake.flower_down();
                    intake.turn_on_intake();
                    setPathState(PathState.HOLD_FLOWER1);
                }
                break;
            case HOLD_FLOWER1:
                if (pathTimer.getElapsedTimeSeconds() > FLOWER_HOLD_SECONDS) {
                    intake.turn_off_intake();
                    intake.flower_up();
                    follower.setMaxPower(1.0);
                    follower.followPath(backupFlower1, true);
                    setPathState(PathState.BACKUP_FLOWER1);
                }
                break;
            case BACKUP_FLOWER1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    flowerPush = false;
                    setTurretDeg(SECOND_HIVE_TURRET_DEG);
                    follower.followPath(toFarSide, true);
                    setPathState(PathState.DRIVE_FAR_SIDE);
                }
                break;
            case DRIVE_FAR_SIDE:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toFarShoot, true);
                    setPathState(PathState.DRIVE_FAR_SHOOT);
                }
                break;
            case DRIVE_FAR_SHOOT:
                if (!follower.isBusy() && turretAimed()) {
                    shootWithFlowerUp();
                    setPathState(PathState.SHOOT_FAR);
                }
                break;
            case SHOOT_FAR:
                if (depo.isDepositOn()) {
                    break;
                }
                if (shotDone()) {
                    follower.setMaxPower(1.0);
                    flowerPush = false;
                    intake.flower_up();
                    follower.followPath(toFlower2Turnaround, true);
                    setPathState(PathState.DROP_FOR_FLOWER2);
                }
                break;
            case DROP_FOR_FLOWER2:
                if (follower.isBusy()) {
                    break;
                }
                if (!flower2DropStarted) {
                    follower.setMaxPower(0.6);
                    flowerPush = false;
                    intake.flower_down();
                    intake.turn_on_intake();
                    flower2DropStarted = true;
                    pathTimer.resetTimer();
                    break;
                }
                if (pathTimer.getElapsedTimeSeconds() < FLOWER2_DROP_SECONDS) {
                    intake.flower_down();
                    intake.turn_on_intake();
                    break;
                }
                follower.setMaxPower(FAR_FLOWER_APPROACH_POWER);
                follower.followPath(toFlower2, true);
                setPathState(PathState.DRIVE_FLOWER2);
                break;
            case BACK_TO_TURNAROUND:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    setTurretDeg(SECOND_HIVE_TURRET_DEG);
                    follower.followPath(backToFarShoot, true);
                    setPathState(PathState.BACK_TO_FAR_SHOOT);
                }
                break;
            case BACK_TO_FAR_SHOOT:
                if (!follower.isBusy() && turretAimed()) {
                    shootWithFlowerUp();
                    setPathState(PathState.SHOOT_RETURN);
                }
                break;
            case SHOOT_RETURN:
                if (shotDone()) {
                    intake.flower_up();
                    intake.turn_off_intake();
                    follower.setMaxPower(1.0);
                    flowerPush = false;
                    follower.followPath(toPark, true);
                    setPathState(PathState.PARK);
                }
                break;
            case DRIVE_FLOWER2:
                if (!flower2Slowing && distanceTo(flower2) <= FAR_FLOWER_APPROACH_INCHES) {
                    flower2Slowing = true;
                    intake.flower_down();
                    follower.setMaxPower(FAR_FLOWER_APPROACH_POWER);
                }
                if (!reachedFlower(flower2Slowing)) {
                    break;
                }
                if (!flower2HoldStarted) {
                    flower2HoldStarted = true;
                    pathTimer.resetTimer();
                    follower.setMaxPower(1.0);
                    intake.flower_down();
                    intake.turn_on_intake();
                    break;
                }
                if (pathTimer.getElapsedTimeSeconds() < FLOWER_HOLD_SECONDS) {
                    intake.flower_down();
                    intake.turn_on_intake();
                    break;
                }
                intake.flower_up();
                intake.turn_off_intake();
                follower.setMaxPower(1.0);
                follower.followPath(backToTurnaround, true);
                setPathState(PathState.BACK_TO_TURNAROUND);
                break;
            case PARK:
                if (pathDone(parkSpot)) {
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
        if (!slowApproach || follower.getVelocity().getMagnitude() > STALL_SPEED) {
            stallTimer.resetTimer();
            return false;
        }
        if (!flowerPush) {
            if (stallTimer.getElapsedTimeSeconds() > STALL_PUSH_SECONDS) {
                flowerPush = true;
                follower.setMaxPower(FLOWER_PUSH_POWER);
                stallTimer.resetTimer();
            }
            return false;
        }
        if (stallTimer.getElapsedTimeSeconds() > STALL_AT_FLOWER_SECONDS) {
            follower.breakFollowing();
            return true;
        }
        return false;
    }

    private boolean pathDone(Pose target) {
        if (!follower.isBusy()) {
            return true;
        }
        double speed = follower.getVelocity().getMagnitude();
        if (follower.getCurrentTValue() >= 0.8 && speed < 2.0 && distanceTo(target) <= 8) {
            follower.breakFollowing();
            return true;
        }
        return false;
    }

    private double distanceTo(Pose target) {
        Pose pose = follower.getPose();
        double dx = target.getX() - pose.getX();
        double dy = target.getY() - pose.getY();
        return Math.hypot(dx, dy);
    }

    /** True once the robot is about {@code seconds} away from the pose at its current speed. */
    private boolean leadTimeReached(Pose target, double seconds) {
        double speed = follower.getVelocity().getMagnitude();
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
        pathTimer = new Timer();
        opModeTimer = new Timer();
        stallTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setCentripetalScaling(0.0015);
        follower.getConstants().setBEZIER_CURVE_SEARCH_LIMIT(100);
        intake = new intakeLM0(hardwareMap);
        intake.flower_up();
        depo = new depoLM0(hardwareMap);
        turret = new turret(hardwareMap);
        timers = new org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer();

        buildPaths();
        follower.setPose(start);
        pathState = PathState.SHOOT_PRELOAD;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        intake.start();
        depo.start();
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
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Turret target", turretTargetDeg);
        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.update();
    }
}
