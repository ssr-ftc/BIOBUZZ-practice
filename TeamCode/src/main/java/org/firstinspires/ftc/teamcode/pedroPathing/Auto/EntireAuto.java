package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.AprilTagStuff.APRILTAG_Biobuzz.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.AprilTagStuff.APRILTAG_Biobuzz.AllianceConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.AprilTagStuff.APRILTAG_Biobuzz.BioBuzzVision;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.AprilTagStuff.APRILTAG_Biobuzz.BioBuzzVisionConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.AprilTagStuff.APRILTAG_Biobuzz.CellOrientation;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.AprilTagStuff.APRILTAG_Biobuzz.FieldSide;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.AprilTagStuff.APRILTAG_Biobuzz.HiveCell;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Depo;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;

/**
 * Full auto. The poses are fixed waypoints. After the garden intake, the
 * audience-side hive of the alliance chosen in init decides which waypoints
 * come next.
 */
@Autonomous(name = "EntireAuto", group = "Season")
public class EntireAuto extends OpMode {
    private static final double FULL_INTAKE_BALLS = 4;
    /** Open the hatch this long after the flywheel is at speed. */
    private static final double SHOT_OPEN_SECONDS = 0.3;
    /** How long every shot runs once the flywheel is at speed. */
    private static final double SHOOT_HOLD_SECONDS = 2;
    /** How long a flower intake stays down. */
    private static final double INTAKE_HOLD_SECONDS = 1;
    /** How long to watch the hive before choosing the next path. */
    private static final double HIVE_LOOK_SECONDS = 0.5;

    /** Field headings. Converted to robot-relative when commanded. */
    private static final double PRELOAD_FIELD_DEG = 63;
    private static final double GARDEN_FIELD_DEG = 41;
    private static final double SHOOT1_FIELD_DEG = 63;
    private static final double OPP_FIELD_DEG = 296;

    private enum PathState {
        PRELOAD_SHOOT,
        TO_GARDEN_APPROACH,
        GARDEN_INTAKE,
        READ_HIVE,
        TO_SHOOT1,
        SHOOT1,
        TO_FLOWER_APPROACH,
        FLOWER_INTAKE,
        TO_FLOWER_BACK,
        TO_OPP_FLOWER,
        SHOOT_HELD,
        INTAKE_OPP,
        SHOOT_OPP,
        TO_PARK_PREP,
        PARK,
        DONE
    }

    private final PoseFactory poses = PoseFactory.degrees();
    private final Pose start = poses.of(36, 6.5, 180);
    private final Pose gardenApproach = poses.of(24, 6.5, 180);
    private final Pose gardenIntakeStart = poses.of(24, 6.5, 180);
    private final Pose gardenIntake = poses.of(10, 6.5, 180);
    private final Pose shoot1Start = poses.of(10, 6.5, 180);
    private final Pose shoot1 = poses.of(36, 6.5, 180);
    private final Pose flowerApproach = poses.of(24, 48, 180);
    private final Pose flowerIntake = poses.of(16, 48, 180);
    private final Pose flowerBack = poses.of(24, 48, 180);
    private final Pose oppFlowerApproachStart = poses.of(24, 48, 90);
    private final Pose oppFlowerApproach = poses.of(42.3663, 128.1642, 75);
    private final Pose parkPrepStart = poses.of(42.3663, 128.1642, 75);
    private final Pose parkPrep = poses.of(18, 96, 0);
    private final Pose park = poses.of(10, 96, 0);

    private Follower follower;
    private Intake intake;
    private Depo depo;
    private Turret turret;
    private Timer timers;
    private final BioBuzzVision vision = new BioBuzzVision();

    private PathState pathState = PathState.PRELOAD_SHOOT;
    private boolean intakeFull;
    private boolean cameraReady;
    private String cameraMessage = "";
    private boolean hiveChecked;
    private boolean hiveFacingUp;
    private CellOrientation seenAudience = CellOrientation.UNKNOWN;
    private double commandedFieldDeg = Double.NaN;

    @Override
    public void init() {
        AllianceConfig.reset();
        follower = Robot1.createFollower(hardwareMap);
        follower.setPose(start);
        intake = new Intake(hardwareMap);
        depo = new Depo(hardwareMap);
        turret = new Turret(hardwareMap);
        timers = new Timer();
        timers.createNew("hold");
        timers.createNew("shoot");
        timers.createNew("look");

        intake.start();
        depo.start();
        turret.resetTurretEncoders();
        turret.setPositionDeg(0);
        intake.flower_up();
        follower.update();

        try {
            vision.init(hardwareMap, telemetry);
            cameraReady = true;
        } catch (RuntimeException e) {
            cameraReady = false;
            cameraMessage = e.getMessage();
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.y) {
            AllianceConfig.select(Alliance.BLUE);
        } else if (gamepad1.a) {
            AllianceConfig.select(Alliance.RED);
        }

        follower.update();
        turret.run_using_pid();
        updateVision();

        telemetry.addLine("Y = BLUE, A = RED");
        telemetry.addData("Alliance", allianceLabel());
        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.addData("Camera", cameraReady ? "ON" : cameraMessage);
        telemetry.update();
    }

    @Override
    public void start() {
        AllianceConfig.lock();
        intake.start();
        depo.start();
        turret.start();
        intake.resetBalls();
    }

    @Override
    public void loop() {
        follower.update();
        turret.run_using_pid();
        depo.run_using_pid();
        updateVision();
        updateIntakeCount();
        autonomousPathUpdate();

        telemetry.addData("Path", pathState);
        telemetry.addData("Alliance", allianceLabel());
        telemetry.addData("Hive", hiveLabel());
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Field aim", Double.isNaN(commandedFieldDeg) ? "hold" : commandedFieldDeg);
        telemetry.addData("Turret deg", turret.getPositionDeg());
        telemetry.addData("Turret target", turret.getTargetDeg());
        telemetry.addData("Depo velo", depo.getVelocity());
        telemetry.addData("Balls", intake.getBalls());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopCollecting();
        intake.flower_up();
        depo.close_hatch();
        depo.turn_off_deposit();
        turret.turretOff();
        if (cameraReady) {
            vision.stop();
        }
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case PRELOAD_SHOOT:
                aimFieldDegrees(PRELOAD_FIELD_DEG);
                if (finishShot()) {
                    follower.follow(line(start, gardenApproach).linear(start, gardenApproach));
                    pathState = PathState.TO_GARDEN_APPROACH;
                }
                break;
            case TO_GARDEN_APPROACH:
                if (!follower.isBusy()) {
                    aimFieldDegrees(GARDEN_FIELD_DEG);
                    collectGarden();
                    follower.follow(line(gardenApproach, gardenIntake).linear(gardenIntakeStart, gardenIntake));
                    pathState = PathState.GARDEN_INTAKE;
                }
                break;
            case GARDEN_INTAKE:
                aimFieldDegrees(GARDEN_FIELD_DEG);
                collectGarden();
                if (!follower.isBusy()) {
                    stopCollecting();
                    seenAudience = CellOrientation.UNKNOWN;
                    pathState = PathState.READ_HIVE;
                }
                break;
            case READ_HIVE:
                rememberAudienceHive();
                if (timers.checkSecondsLast("look", HIVE_LOOK_SECONDS)) {
                    hiveChecked = true;
                    hiveFacingUp = seenAudience == CellOrientation.FACING_UP;
                    if (hiveFacingUp) {
                        follower.follow(line(gardenIntake, shoot1).linear(shoot1Start, shoot1));
                        pathState = PathState.TO_SHOOT1;
                    } else {
                        follower.follow(line(gardenIntake, oppFlowerApproach).linear(gardenIntake, oppFlowerApproach));
                        pathState = PathState.TO_OPP_FLOWER;
                    }
                } else if (!timers.isRunning("look")) {
                    timers.start("look");
                }
                break;
            case TO_SHOOT1:
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT1;
                }
                break;
            case SHOOT1:
                aimFieldDegrees(SHOOT1_FIELD_DEG);
                if (finishShot()) {
                    follower.follow(line(shoot1, flowerApproach).constant(flowerApproach));
                    pathState = PathState.TO_FLOWER_APPROACH;
                }
                break;
            case TO_FLOWER_APPROACH:
                if (!follower.isBusy()) {
                    prepareFlower();
                    follower.follow(line(flowerApproach, flowerIntake).constant(flowerIntake));
                    pathState = PathState.FLOWER_INTAKE;
                }
                break;
            case FLOWER_INTAKE:
                if (timers.checkSecondsLast("hold", INTAKE_HOLD_SECONDS)) {
                    intake.flower_up();
                    stopCollecting();
                    follower.follow(line(flowerIntake, flowerBack).constant(flowerBack));
                    pathState = PathState.TO_FLOWER_BACK;
                } else if (!follower.isBusy() && !timers.isRunning("hold")) {
                    intake.resetBalls();
                    intakeFull = false;
                    collectFlower();
                    timers.start("hold");
                }
                break;
            case TO_FLOWER_BACK:
                if (!follower.isBusy()) {
                    follower.follow(line(flowerBack, oppFlowerApproach)
                            .linear(oppFlowerApproachStart, oppFlowerApproach));
                    pathState = PathState.TO_OPP_FLOWER;
                }
                break;
            case TO_OPP_FLOWER:
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_HELD;
                }
                break;
            case SHOOT_HELD:
                aimFieldDegrees(OPP_FIELD_DEG);
                if (finishShot()) {
                    prepareFlower();
                    pathState = PathState.INTAKE_OPP;
                }
                break;
            case INTAKE_OPP:
                if (timers.checkSecondsLast("hold", INTAKE_HOLD_SECONDS)) {
                    intake.flower_up();
                    stopCollecting();
                    pathState = PathState.SHOOT_OPP;
                } else if (!timers.isRunning("hold")) {
                    intake.resetBalls();
                    intakeFull = false;
                    collectFlower();
                    timers.start("hold");
                }
                break;
            case SHOOT_OPP:
                aimFieldDegrees(OPP_FIELD_DEG);
                if (finishShot()) {
                    follower.follow(line(oppFlowerApproach, parkPrep).linear(parkPrepStart, parkPrep));
                    pathState = PathState.TO_PARK_PREP;
                }
                break;
            case TO_PARK_PREP:
                if (!follower.isBusy()) {
                    follower.follow(line(parkPrep, park).constant(park));
                    pathState = PathState.PARK;
                }
                break;
            case PARK:
                if (!follower.isBusy()) {
                    pathState = PathState.DONE;
                }
                break;
            case DONE:
                break;
            default:
                break;
        }
    }

    /**
     * Field heading to the robot-relative angle {@link Turret#setPositionDeg(double)} expects.
     * Same conversion as {@link Turret#AutoAllignTo}: -(field - robot heading).
     */
    private void aimFieldDegrees(double fieldDeg) {
        commandedFieldDeg = fieldDeg;
        double headingDeg = Math.toDegrees(follower.pose().heading());
        turret.setPositionDeg(-(fieldDeg - headingDeg));
    }

    /** One shot. True on the loop it finishes and the flywheel is turned off. */
    private boolean finishShot() {
        if (!depo.isDepositOn() && !timers.isRunning("shoot")) {
            depo.turn_on_deposit();
        }
        if (depo.reachedvelocity() && !timers.isRunning("shoot")) {
            timers.start("shoot");
        }
        if (timers.checkSeconds("shoot", SHOT_OPEN_SECONDS)) {
            intake.intake_on();
            depo.open_hatch();
        }
        if (timers.checkSecondsLast("shoot", SHOOT_HOLD_SECONDS)) {
            depo.close_hatch();
            depo.turn_off_deposit();
            intake.intake_off();
            intake.resetBalls();
            intakeFull = false;
            return true;
        }
        return false;
    }

    private void updateVision() {
        if (cameraReady) {
            vision.update(getRuntime());
        }
    }

    /** Audience cell of the alliance picked in init. The other alliance is ignored. */
    private void rememberAudienceHive() {
        Alliance alliance = AllianceConfig.get();
        if (alliance == null) {
            return;
        }
        for (HiveCell cell : vision.getVisibleCells()) {
            if (cell.alliance == alliance
                    && cell.fieldSide == FieldSide.AUDIENCE
                    && cell.orientation != CellOrientation.UNKNOWN
                    && cell.meanDecisionMargin >= BioBuzzVisionConfig.MIN_MARGIN_FOR_ORIENTATION) {
                seenAudience = cell.orientation;
            }
        }
    }

    private void updateIntakeCount() {
        if (shooting() || !intake.intaking() || intakeFull) {
            return;
        }
        intake.countBalls();
        if (intake.getBalls() >= FULL_INTAKE_BALLS) {
            intakeFull = true;
            stopCollecting();
        }
    }

    private boolean shooting() {
        return pathState == PathState.PRELOAD_SHOOT
                || pathState == PathState.SHOOT1
                || pathState == PathState.SHOOT_HELD
                || pathState == PathState.SHOOT_OPP;
    }

    private void collectGarden() {
        if (intakeFull) {
            return;
        }
        intake.flower_up();
        intake.front_down();
        intake.back_down();
        intake.intake_on();
    }

    private void prepareFlower() {
        intake.front_up();
        intake.back_up();
        intake.flower_down();
        intake.intake_off();
    }

    private void collectFlower() {
        if (intakeFull) {
            return;
        }
        intake.front_up();
        intake.back_up();
        intake.flower_down();
        intake.intake_on();
    }

    private void stopCollecting() {
        intake.intake_off();
        intake.front_up();
        intake.back_up();
    }

    private String allianceLabel() {
        Alliance alliance = AllianceConfig.get();
        if (alliance == null) {
            return "NONE - press Y or A";
        }
        return alliance.name();
    }

    private String hiveLabel() {
        if (!hiveChecked) {
            return seenAudience.name();
        }
        return hiveFacingUp ? "FACING UP" : "NOT FACING UP";
    }
}
