package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

//import com.pedropathing.ivy.Scheduler;
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.groups.Groups.sequential;
//import static com.pedropathing.ivy.pedro.PedroCommands.*;


import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

//import org.firstinspires.ftc.teamcode.OFSB2.Subsystems.turret;
//import org.firstinspires.ftc.teamcode.OFSB2.Subsystems.intake;
//import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.AprilTagStuff.APRILTAG_Biobuzz.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.AprilTagStuff.APRILTAG_Biobuzz.AprilTagDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.AprilTagStuff.APRILTAG_Biobuzz.BioBuzzVisionConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.AprilTagStuff.APRILTAG_Biobuzz.HiveCluster;
import org.firstinspires.ftc.teamcode.pedroPathing.Vision.AprilTagStuff.APRILTAG_Biobuzz.TagClusterMapper;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.CustomFollower;

import java.util.List;
//import static com.pedropathing.ivy.pedro.PedroCommands.*;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.depoLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.intakeLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.soloTeleop;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.turret;


@Autonomous(name = "AutoLM0", group = "Season")
public class LM0_Auto extends OpMode {

    soloTeleop teleop = new soloTeleop();
    //boolean needDepo = true;
    private DcMotorEx depo1;
    private depoLM0 depo;
    private intakeLM0 intake;
    private Servo hatch;
    private CRServo turret_servo;
    private turret turret;
    private CustomFollower follower;
    Pose start, sample1, shoot, sample2, done;
    private PathChain startToSample1, sample1ToShoot, shootToSample2, sample2ToDone;

    private Timer pathTimer, opModeTimer;
    // pedroPathing's Timer is already imported above, so the team's own Timer (same one soloTeleop uses) needs its full name
    private org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer timers;

    boolean isBlue, isRed, isSmallBlue, isSmallRed, isBlueSample, isRedSample;
    //intake intake;
    // turret turret;

    // soloTeleop teleop = new soloTeleop();

    private enum AutoState {
        PRESHOOT,
        STARTTOSAMPLE1,
        SAMPLE1TOSHOOT,
        SHOOTTOSAMPLE2,
        SAMPLE2TODONE,
        DONE
    }

    private enum ShootState {
        INTAKE,
        SHOOT,
        TURRET,
        KICK;
    }

    private AutoState autoState;
    private ShootState shootState;
    private boolean intaking = false;
    private boolean depositRunning = false;
    private boolean pausedAtSample2 = false;
    private boolean preshootStarted = false;
    private boolean hatchOpened = false;
    private static final double TURRET_TOLERANCE_DEG = 3.0;

    // ---------------- APRILTAG (read during INIT) ----------------
    private final AprilTagDetector aprilTag = new AprilTagDetector();
    private boolean cameraStarted = false;       // camera was created without an error
    private String cameraMessage = "";

    private AprilTagDetection seenTag = null;    // best HIVE tag the camera saw during INIT (null = none yet)
    private HiveCluster seenHive = null;         // which hive cell that tag belongs to
    private boolean otherSideHiveAssumed = true; // true while no tag has been seen
    private Alliance tagAlliance = null;         // null = accept every hive tag; RED / BLUE = only that alliance's tags

    // What we assume about the other side's hive if INIT ends without ever seeing a tag.
    // PLACEHOLDER word - change it to whatever you meant by "u..".
    private static final String HIVE_IF_NO_TAG = "UNKNOWN";
    private static final long LOST_TAG_GRACE_NS = 1_000_000_000L;
    private HiveCluster selectedHive = null;
    private HiveCluster oppositeHive = null;
    private long lastVisibleTagNanos = 0;
    private boolean switchedToOpposite = false;
    private int rawDetectionCount = 0;
    private int blueDetectionCount = 0;
    private int redDetectionCount = 0;
    private int unmappedDetectionCount = 0;
    private int badHammingCount = 0;
    private boolean previousBothBumpers = false;
    public void subsystems() {
        //intake = new intake(hardwareMap);
        // turret = new turret(hardwareMap);
    }

    @Override
    public void init() {
        //Scheduler.reset();
        subsystems();
        pathTimer = new Timer();
        opModeTimer = new Timer();
        timers = new org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer();
        follower = new CustomFollower(hardwareMap, telemetry);
        intake = new intakeLM0(hardwareMap);
        depo = new depoLM0(hardwareMap);
        turret = new turret(hardwareMap);
        initAprilTag();
        telemetry.addLine("Ready. Press Y for Blue, A for Red.");
        telemetry.addLine("Press X for Small Blue, B for Small Red.");
        telemetry.addLine("Press D-pad UP for Blue Sample, D-pad DOWN for Red Sample.");
    }

    @Override
    public void init_loop() {
        gamepadSide();
        selectTagAlliance();
        updateAprilTag();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        intake.start();
        depo.start();
        turret.ResetTurretEncoders();
        timers.createNew("depo");
    }

    @Override
    public void loop() {
        follower.update();
        updateAutoState();
        updateMechanisms();
        updateAprilTag(); // Continue reading the camera after START.
        updateTelemetry();
    }

    @Override
    public void stop() {
        if (cameraStarted) {
            aprilTag.stop();
        }
    }

    /** During INIT: left bumper = BLUE, right bumper = RED, both = ALL. */
    private void selectTagAlliance() {
        Alliance picked = tagAlliance;
        boolean both = gamepad1.left_bumper && gamepad1.right_bumper;
        if (both && !previousBothBumpers) {
            picked = null;
        } else if (!both && gamepad1.left_bumper) {
            picked = Alliance.BLUE;
        } else if (!both && gamepad1.right_bumper) {
            picked = Alliance.RED;
        }
        previousBothBumpers = both;
        if (picked != tagAlliance) {
            tagAlliance = picked;
            seenTag = null;
            seenHive = null;
            selectedHive = null;
            oppositeHive = null;
            switchedToOpposite = false;
            lastVisibleTagNanos = 0;
        }
    }

    private void initAprilTag() {
        try {
            aprilTag.init(hardwareMap, telemetry);
            cameraStarted = true;
        } catch (Exception e) {
            cameraStarted = false;
            cameraMessage = "could not start: " + e.getMessage();
        }
    }

    /**
     * Determine the other hive only if the mapper contains EXACTLY two distinct
     * hive cells for the current alliance. Don't guess if mapping is ambiguous.
     */
    private HiveCluster findOppositeHive(HiveCluster current) {
        if (current == null) return null;
        HiveCluster opposite = null;
        for (int id = 30; id <= 45; id++) {
            HiveCluster candidate = TagClusterMapper.getCellForTagId(id);
            if (candidate == null || candidate.alliance != current.alliance
                    || candidate.displayName().equals(current.displayName())) continue;
            if (opposite != null && !opposite.displayName().equals(candidate.displayName())) {
                return null; // More than two cells: cannot safely infer an opposite.
            }
            opposite = candidate;
        }
        return opposite;
    }

    private void updateAprilTag() {
        telemetry.addData("Tag filter (LB blue / RB red / BOTH all)",
                tagAlliance == null ? "ALL" : tagAlliance.name());
        if (!cameraStarted) {
            telemetry.addData("Camera", cameraMessage);
            return;
        }

        aprilTag.update();
        telemetry.addData("Camera streaming", aprilTag.isStreaming());

        List<AprilTagDetection> detections = aprilTag.getDetections();
        rawDetectionCount = detections == null ? 0 : detections.size();
        blueDetectionCount = 0;
        redDetectionCount = 0;
        unmappedDetectionCount = 0;
        badHammingCount = 0;
        AprilTagDetection best = null;
        HiveCluster bestCell = null;

        if (detections != null) for (AprilTagDetection d : detections) {
            telemetry.addData("Raw ID " + d.id,
                    "hamming=%d, margin=%.1f", d.hamming, d.decisionMargin);
            HiveCluster cell = TagClusterMapper.getCellForTagId(d.id);
            if (cell == null) {
                unmappedDetectionCount++;
                continue;
            }
            if (cell.alliance == Alliance.BLUE) blueDetectionCount++;
            else if (cell.alliance == Alliance.RED) redDetectionCount++;

            if (d.hamming > BioBuzzVisionConfig.MAX_HAMMING) {
                badHammingCount++;
                continue;
            }
            if (tagAlliance != null && cell.alliance != tagAlliance) continue;
            if (best == null || d.decisionMargin > best.decisionMargin) {
                best = d;
                bestCell = cell;
            }
        }

        telemetry.addData("Raw tags", rawDetectionCount);
        telemetry.addData("Mapped RED", redDetectionCount);
        telemetry.addData("Mapped BLUE", blueDetectionCount);
        telemetry.addData("Unmapped", unmappedDetectionCount);
        telemetry.addData("Bad hamming", badHammingCount);

        if (best != null) {
            seenTag = best;
            seenHive = bestCell;
            selectedHive = bestCell;
            oppositeHive = findOppositeHive(bestCell);
            lastVisibleTagNanos = System.nanoTime();
            switchedToOpposite = false;
            otherSideHiveAssumed = false;
            telemetry.addData("Current tag", best.id);
            telemetry.addData("Current hive", bestCell.displayName());
            if (best.ftcPose != null) {
                telemetry.addData("Range (in)", "%.1f", best.ftcPose.range);
                telemetry.addData("Bearing (deg)", "%.1f", best.ftcPose.bearing);
                telemetry.addData("Yaw (deg)", "%.1f", best.ftcPose.yaw);
            }
        } else if (seenTag != null) {
            // A previously seen target disappeared: propose switching to the
            // opposite mapped hive after a short dropout tolerance.
            long elapsed = System.nanoTime() - lastVisibleTagNanos;
            if (elapsed >= LOST_TAG_GRACE_NS && oppositeHive != null) {
                selectedHive = oppositeHive;
                switchedToOpposite = true;
            }
            telemetry.addLine("No accepted tag NOW (last tag is remembered)");
        } else {
            telemetry.addLine("AprilTag: NOT seen");
            otherSideHiveAssumed = true;
        }

        telemetry.addData("Last accepted ID", seenTag == null ? "none" : seenTag.id);
        telemetry.addData("Seen hive", seenHive == null ? "UNKNOWN" : seenHive.displayName());
        telemetry.addData("Opposite mapped hive",
                oppositeHive == null ? "UNRESOLVED" : oppositeHive.displayName());
        telemetry.addData("SELECTED hive",
                selectedHive == null ? HIVE_IF_NO_TAG : selectedHive.displayName());
        telemetry.addData("Switched after losing tag", switchedToOpposite);
        // NOTE: selectedHive is decision state only. Mechanical aiming and
        // Pedro paths require hive-specific geometry not supplied here.
    }

    // --- THE SINGLE STATE MACHINE ---
    private void updateAutoState() {
        if (follower.isBusy()) {
            return;
        }

        switch (autoState) {
            case PRESHOOT:
                if (!preshootStarted) {
                    depo.turn_on_deposit();
                    depositRunning = true;
                    turret.setPositionDeg(isBlue ? 90 : -90);
                    preshootStarted = true;
                }
                if (shotFinished()) {
                    intake();
                    follower.acceleration(0.5, 1.0, 1.0, 0.5);
                    follower.followPath(startToSample1, true);
                    setPathState(AutoState.SAMPLE1TOSHOOT);
                }
                break;

            case STARTTOSAMPLE1:
                intake();
                follower.acceleration(0.5, 1.0, 1.0, 0.5);
                follower.followPath(startToSample1, true);
                setPathState(AutoState.SAMPLE1TOSHOOT);
                break;

            case SAMPLE1TOSHOOT:
                stopIntake();
                if (isBlue) {
                    follower.acceleration(0.6, 1.0, 0.4, 0.4);
                }
                follower.followPath(sample1ToShoot, true);
                setPathState(AutoState.SHOOTTOSAMPLE2);
                break;

            case SHOOTTOSAMPLE2:
                if (isBlue) {
                    if (!pausedAtSample2) {
                        intake();
                        pathTimer.resetTimer();
                        pausedAtSample2 = true;
                        return;
                    }
                    if (pathTimer.getElapsedTimeSeconds() < 1.0) {
                        return;
                    }
                    stopIntake();
                    follower.followPath(shootToSample2, true);
                    setPathState(AutoState.SAMPLE2TODONE);
                    break;
                }
                follower.followPath(shootToSample2, true);
                setPathState(AutoState.SAMPLE2TODONE);
                break;

            case SAMPLE2TODONE:
                if (isBlue) {
                    setPathState(AutoState.DONE);
                    break;
                }
                if (isRed || isBlueSample || isRedSample) {
                    follower.followPath(sample2ToDone, true);
                }
                else {
                    follower.followPath(startToSample1, true);
                }
                setPathState(AutoState.DONE);
                break;

            case DONE:
                telemetry.addLine("Finished everything!");
                break;

            default:
                break;
        }
    }

    private void intake() {
        intaking = true;
    }

    private void stopIntake() {
        intaking = false;
        intake.turn_off_intake();
    }

    /**
     * Same shot as LM0Teleop: once the turret is on target and the deposit is at speed,
     * open the hatch and run the intake, then close the hatch and stop the deposit after 2 seconds.
     * Returns true on the loop the shot finishes.
     */
    private boolean shotFinished() {
        if (turret.atPositionDeg(TURRET_TOLERANCE_DEG)
                && depo.reachedvelocity()
                && !timers.isRunning("depo")
                && !hatchOpened) {
            timers.start("depo");
            depo.open_hatch(turret.getPositionDeg());
            intake();
            hatchOpened = true;
        }
        if (timers.checkSecondsLast("depo", 2)) {
            depo.turn_off_deposit();
            depo.close_hatch();
            depositRunning = false;
            return true;
        }
        return false;
    }

    private void updateMechanisms() {
        if (intaking) {
            intake.turn_on_intake();
        }
        if (depositRunning) {
            depo.run_using_pid();
        }
        turret.run_using_pid();
    }

    private void setPathState(AutoState newState) {
        autoState = newState;
        pathTimer.resetTimer();
    }

    private void setShootState(ShootState newState) {
        shootState = newState;
    }

   /* public void hardwareMap() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        depo = hardwareMap.get(DcMotorEx.class, "depo");
        depo1 = hardwareMap.get(DcMotorEx.class, "depo1");

        hatch = hardwareMap.get(Servo.class, "hatch");
    }

    */

    private void poses() {
        if (isBlue) {
            start = new Pose(84.3585, 134.3146, Math.toRadians(0));
            sample1 = new Pose(126.2366, 134.7049, Math.toRadians(0));
            sample2 = new Pose(100.7233, 23.8835, Math.toRadians(-90));
            done = new Pose(138.7167, 33.8492, Math.toRadians(180));
        } else if (isRed) {
            start = new Pose(59.64146341463415, 9.68536585365854, Math.toRadians(180));
            sample1 = new Pose(17.76341463414636, 9.29512195121953, Math.toRadians(180));
            shoot = new Pose(53.89512195121951, 107.25609756097561, Math.toRadians(90));
            sample2 = new Pose(43.27665036674817, 127.11651857594373, Math.toRadians(90));
            done = new Pose(5.283308485896602, 110.15084381895165, Math.toRadians(0));
        } else if (isSmallBlue) {
            start = new Pose(80.21707317073172, 9.035365853658533, Math.toRadians(90));
            done = new Pose(139.17439024390245, 41.521951219512204, Math.toRadians(180));
        } else if (isSmallRed) {
            start = new Pose(63.78292682926828, 134.96463414634147, Math.toRadians(270));
            done = new Pose(4.82560975609755, 102.4780487804878, Math.toRadians(0));
        } else if (isBlueSample) {
            start = new Pose(80.2171, 9.0354, Math.toRadians(-90));
            sample1 = new Pose(94.3909, 16.2493, Math.toRadians(-90));
            done = new Pose(139.1744, 29.522, Math.toRadians(180));
        } else if (isRedSample) {
            // Exact mathematical 144-reflection of the Blue Sample poses
            start = new Pose(63.7829, 134.9646, Math.toRadians(90));
            sample1 = new Pose(49.6091, 127.7507, Math.toRadians(90));
            done = new Pose(4.8256, 114.4780, Math.toRadians(0));
        }
    }

    private void buildPaths() {
        if (isBlue) {
            startToSample1 = follower.pathBuilder()
                    .addPath(new BezierLine(start, sample1))
                    .setLinearHeadingInterpolation(start.getHeading(), sample1.getHeading())
                    .build();

            sample1ToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(sample1, new Pose(94.8854, 25.9476), sample2))
                    .setHeadingInterpolation(HeadingInterpolator.piecewise(
                            HeadingInterpolator.PiecewiseNode.linear(0, 0.15, Math.toRadians(0), Math.toRadians(-90)),
                            new HeadingInterpolator.PiecewiseNode(0.15, 1, HeadingInterpolator.constant(Math.toRadians(-90)))
                    ))
                    .build();

            shootToSample2 = follower.pathBuilder()
                    .addPath(new BezierCurve(sample2, new Pose(98.3366, 38.0268), done))
                    .setLinearHeadingInterpolation(sample2.getHeading(), done.getHeading())
                    .build();
        }
        else if(isRed) {
            startToSample1 = follower.pathBuilder()
                    .addPath(new BezierLine(start, sample1))
                    .setLinearHeadingInterpolation(start.getHeading(), sample1.getHeading())
                    .build();

            sample1ToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(sample1, new Pose(18.94756097560975, 78.21219512195121), new Pose(31.38170731707316, 114.09634146341463), shoot))
                    .setLinearHeadingInterpolation(sample1.getHeading(), shoot.getHeading())
                    .build();

            shootToSample2 = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot, new Pose(49.11463414634146, 118.05243902439024), sample2))
                    .setLinearHeadingInterpolation(shoot.getHeading(), sample2.getHeading())
                    .build();

            sample2ToDone = follower.pathBuilder()
                    .addPath(new BezierCurve(sample2, new Pose(45.66341463414635, 105.97317073170731), done))
                    .setLinearHeadingInterpolation(sample2.getHeading(), done.getHeading())
                    .build();
        }
        else if(isSmallBlue) {
            startToSample1 = follower.pathBuilder()
                    .addPath(new BezierCurve(start, new Pose(73.47804878048782, 23.213414634146346), done))
                    .setLinearHeadingInterpolation(start.getHeading(), done.getHeading())
                    .build();

        }
        else if(isSmallRed) {
            startToSample1 = follower.pathBuilder()
                    .addPath(new BezierCurve(start, new Pose(70.52195121951218, 120.78658536585365), done))
                    .setLinearHeadingInterpolation(start.getHeading(), done.getHeading())
                    .build();
        }
        else if(isBlueSample) {
            shootToSample2 = follower.pathBuilder()
                    .addPath(new BezierCurve(start, new Pose(88.3183, 28.3902), sample1))
                    .setLinearHeadingInterpolation(start.getHeading(), sample1.getHeading())
                    .build();

            sample2ToDone = follower.pathBuilder()
                    .addPath(new BezierLine(sample1, done))
                    .setLinearHeadingInterpolation(sample1.getHeading(), done.getHeading())
                    .build();
        }
        else if(isRedSample) {
            shootToSample2 = follower.pathBuilder()
                    // Exactly reflected control point (144 - X, 144 - Y)
                    .addPath(new BezierCurve(start, new Pose(55.6817, 115.6098), sample1))
                    .setLinearHeadingInterpolation(start.getHeading(), sample1.getHeading())
                    .build();

            sample2ToDone = follower.pathBuilder()
                    .addPath(new BezierLine(sample1, done))
                    .setLinearHeadingInterpolation(sample1.getHeading(), done.getHeading())
                    .build();
        }
    }

    public void gamepadSide() {
        if (gamepad1.y) {
            isBlue = true; isRed = false; isSmallBlue = false; isSmallRed = false; isBlueSample = false; isRedSample = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.PRESHOOT;
            telemetry.addData("Selected", "BLUE");
        }
        else if(gamepad1.a) {
            isRed = true; isBlue = false; isSmallBlue = false; isSmallRed = false; isBlueSample = false; isRedSample = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.PRESHOOT;
            telemetry.addData("Selected", "RED");
        }
        else if(gamepad1.x) {
            isSmallBlue = true; isBlue = false; isRed = false; isSmallRed = false; isBlueSample = false; isRedSample = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.SAMPLE2TODONE;
            telemetry.addData("Selected", "SMALL BLUE");
        }
        else if(gamepad1.b) {
            isSmallRed = true; isSmallBlue = false; isBlue = false; isRed = false; isBlueSample = false; isRedSample = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.SAMPLE2TODONE;
            telemetry.addData("Selected", "SMALL RED");
        }
        else if(gamepad1.dpad_up) {
            isBlueSample = true; isBlue = false; isRed = false; isSmallBlue = false; isSmallRed = false; isRedSample = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.SHOOTTOSAMPLE2;
            telemetry.addData("Selected", "BLUE SAMPLE");
        }
        else if(gamepad1.dpad_down) {
            isRedSample = true; isBlue = false; isRed = false; isSmallBlue = false; isSmallRed = false; isBlueSample = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.SHOOTTOSAMPLE2;
            telemetry.addData("Selected", "RED SAMPLE");
        }
    }

    private void updateTelemetry() {
        telemetry.addData("auto state", autoState);
        telemetry.addData("turret deg", turret.getPositionDeg());
        Pose pose = follower.getPose();
        if (pose != null) {
            telemetry.addData("x", pose.getX());
            telemetry.addData("y", pose.getY());
            telemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
        }
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());

        telemetry.addData("Tag filter", tagAlliance == null ? "ALL" : tagAlliance.name());
        telemetry.addData("Last accepted tag", seenTag == null ? "none" : seenTag.id);
        telemetry.addData("Selected hive", selectedHive == null ? "UNKNOWN" : selectedHive.displayName());
        telemetry.addData("Opposite hive", oppositeHive == null ? "UNRESOLVED" : oppositeHive.displayName());
    }
}
