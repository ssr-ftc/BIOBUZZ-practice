package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

//import com.pedropathing.ivy.Scheduler;
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.groups.Groups.sequential;
//import static com.pedropathing.ivy.pedro.PedroCommands.*;


import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Timer;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
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
import java.util.List;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.CustomFollower;
//import static com.pedropathing.ivy.pedro.PedroCommands.*;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Depo;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;


@Autonomous(name = "EntireAuto", group = "Season")
public class EntireAuto extends OpMode {
    //boolean needDepo = true;
    private DcMotorEx depo1;
    private Depo depo;
    private Intake intake;
    private Turret turret;
    private Servo hatch;
    private CRServo turret_servo;
    private CustomFollower follower;
    Pose start, sample1, point2, sample2, sample3, done;
    Pose point4, point5, doneStart;
    Pose sample3Control1, sample2Control1, sample2Control2;
    Pose sample1Segment1Start, sample1Segment1End, sample1Segment2Start, sample1Segment2End;
    static Pose teleop;
    private Path shootToSample1, sample1ToPoint2, point2ToSample2, sample2ToShoot2, point4ToPoint5, sample3ToDone;

    private Timer pathTimer, opModeTimer;
    // pedroPathing's Timer is already imported above, so the team's own Timer (same one soloTeleop uses) needs its full name
    private org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer timers;

    static boolean isBlue;
    static boolean isRed;
    boolean isHatchClosed;
    boolean isShootOffset;
    boolean isBlueGarden, isBlueOpposite, isRedGarden, isRedOpposite;
    // True once that drive has been started, so the case does not restart the path when the follower goes idle.
    private boolean shootToSample1Started = false;
    private boolean sample1ToPoint2Started = false;
    private boolean point2ToSample2Started = false;
    private boolean sample2ToShoot2Started = false;
    private boolean shoot2ToSample3Started = false;
    // False until the flower path is moving, then the flower stays down until that path ends.
    private boolean flowerPathStarted = false;
    private boolean flowerIntakeDone;
    //intake intake;
    // turret turret;

    // soloTeleop teleop = new soloTeleop();

    private enum AutoState {
        SHOOTTOSAMPLE1,
        SAMPLE1TOPOINT2,
        POINT2TOSAMPLE2,
        SAMPLE2TOSHOOT2,
        SHOOT2TOSAMPLE3,
        SAMPLE3TODONE,
        DONE;
    }

    private enum ShootState {
        INTAKE,
        SHOOT,
        SHOOTOFFSET,
        FLOWERINTAKE;
    }

    private AutoState autoState;
    private ShootState shootState;
    // Close enough to the commanded turret angle to open the hatch.
    private static final double TURRET_TOLERANCE_DEG = 3.0;
    // ---------------- APRILTAG (read during INIT) ----------------
    private final AprilTagDetector aprilTag = new AprilTagDetector();
    private boolean cameraStarted = false;       // ca  mera was created without an error
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
        intake = new Intake(hardwareMap);
        depo = new Depo(hardwareMap);
        turret = new Turret(hardwareMap);
        initAprilTag();
        telemetry.addLine("READY. PRESS Y FOR BLUE, A FOR RED.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        gamepadSide();
        selectTagAlliance();
        telemetry.addLine("Y = BLUE, A = RED.");
        telemetry.addData("SELECTED", selectedScenario());
        updateAprilTag();
    }

    @Override
    public void start() {
        opModeTimer.reset();
        pathTimer.reset();
        intake.start();
        depo.start();
        turret.holdCurrentPosition();
        timers.createNew("depo");
        shootToSample1Started = false;
        sample1ToPoint2Started = false;
        point2ToSample2Started = false;
        sample2ToShoot2Started = false;
        shoot2ToSample3Started = false;
        flowerPathStarted = false;
        flowerIntakeDone = false;
        setShootState(ShootState.SHOOTOFFSET);
        isShootOffset = true;
        turnOnShooter();
    }

    @Override
    public void loop() {
        teleop = new Pose(follower.getPose().x(), follower.getPose().y(), follower.getPose().heading());
        follower.update();
        turret.run_using_pid();
        depo.run_using_pid();
        updateAutoState();
        telemetry.addData("SELECTED", selectedScenario());
        updateAprilTag();
        updateTelemetry();
        updateShootState();
    }

    @Override
    public void stop() {
        if (cameraStarted) {
            aprilTag.stop();
        }
    }

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
            case SHOOTTOSAMPLE1:
                if (isRed) {
                    isRedGarden = true;
                    isRedOpposite = false;
                } else {
                    isBlueGarden = true;
                    isBlueOpposite = false;
                }
                if (!shootToSample1Started) {
                    isHatchClosed = false;
                    isShootOffset = true;
                    turnOnShooter();
                    setShootState(ShootState.SHOOTOFFSET);
                    shootToSample1Started = true;
                    break;
                }
                if (isHatchClosed) {
                    isHatchClosed = false;
                    setShootState(ShootState.INTAKE);
                    //follower.acceleration(0.5, 1.0, 1.0, 0.5);
                    follower.followPath(shootToSample1, true);
                    setPathState(AutoState.SAMPLE1TOPOINT2);
                }
                break;

            case SAMPLE1TOPOINT2:
                if (isRed) {
                    isRedGarden = true;
                    isRedOpposite = false;
                } else {
                    isBlueGarden = false;
                    isBlueOpposite = true;
                }
                // Drive sample 1 to point 2 once. The next time this case runs, the robot is stopped there.
                if (!sample1ToPoint2Started) {
                    follower.followPath(sample1ToPoint2, true);
                    isShootOffset = true;
                    turnOnShooter();
                    sample1ToPoint2Started = true;
                    break;
                }

                setShootState(ShootState.SHOOTOFFSET);

                if (isHatchClosed) {
                    isHatchClosed = false;
                    sample1ToPoint2Started = false;
                    flowerIntakeDone = false;
                    setShootState(ShootState.FLOWERINTAKE);
                    setPathState(AutoState.POINT2TOSAMPLE2);
                }
                break;

            case POINT2TOSAMPLE2:
                if (!point2ToSample2Started) {
                    follower.followPath(point2ToSample2, true);
                    point2ToSample2Started = true;
                    break;
                }
                if (pathTimer.seconds() >= 3) {
                    setPathState(AutoState.SAMPLE2TOSHOOT2);
                }
                break;

            case SAMPLE2TOSHOOT2:
                if (isRed) {
                    isRedGarden = false;
                    isRedOpposite = true;
                } else {
                    isBlueGarden = false;
                    isBlueOpposite = true;
                }
                if (!sample2ToShoot2Started) {
                    follower.followPath(sample2ToShoot2, true);
                    isShootOffset = true;
                    turnOnShooter();
                    sample2ToShoot2Started = true;
                    break;
                }

                setShootState(ShootState.SHOOTOFFSET);

                if (isHatchClosed) {
                    isHatchClosed = false;
                    sample2ToShoot2Started = false;
                    setShootState(ShootState.INTAKE);
                    setPathState(AutoState.SHOOT2TOSAMPLE3);
                }
                break;

            case SHOOT2TOSAMPLE3:
                if (!shoot2ToSample3Started) {
                    follower.followPath(point4ToPoint5, true);
                    shoot2ToSample3Started = true;
                    break;
                }
                setPathState(AutoState.SAMPLE3TODONE);
                break;

            case SAMPLE3TODONE:
                follower.followPath(sample3ToDone, true);
                setPathState(AutoState.DONE);
                break;

            case DONE:
                telemetry.addLine("Finished everything!");
                break;

            default:
                break;
        }
    }

    private void updateShootState() {
        switch (shootState) {
            case INTAKE:
                //needDepo = false;
                intake.intake_on();
                break;
            case SHOOT:
                isShootOffset = false;
                intake.intake_on();
                turret.setPositionDeg(90);
                if (Math.abs(turret.getPositionDeg() - 90) <= TURRET_TOLERANCE_DEG) {
                    runAutoDepo();
                } else if (timers.isRunning("depo")) {
                    runAutoDepo();
                }
                break;
            case SHOOTOFFSET:
                isShootOffset = true;
                Pose robot = new Pose(follower.getPose().x(), follower.getPose().y(), follower.getPose().heading());

                if (isBlueGarden) {
                    turret.faceGardenHive(robot);
                }
                if (isBlueOpposite) {
                    turret.faceFlowerHive(robot);
                }
                if (isRedGarden) {
                    turret.faceFlowerHive(robot);
                }
                if (isRedOpposite) {
                    turret.faceGardenHive(robot);
                }

                if (Math.abs(turret.getPositionDeg() - Turret.deg) <= TURRET_TOLERANCE_DEG) {
                    runAutoDepo();
                } else if (timers.isRunning("depo")) {
                    runAutoDepo();
                }
                break;
            case FLOWERINTAKE:
                if (pathTimer.seconds() >= 3) {
                    flowerIntakeDone = true;
                }
                if (!flowerIntakeDone) {
                    intake.flower_down();
                    intake.intake_on();
                }
                else {
                    intake.flower_up();
                    intake.intake_off();
                }
                break;

        }
    }

    private void setPathState(AutoState newState) {
        autoState = newState;
        pathTimer.reset();
    }

    private void setShootState(ShootState newState) {
        if (newState == ShootState.FLOWERINTAKE && shootState != ShootState.FLOWERINTAKE) {
            flowerPathStarted = false;
            flowerIntakeDone = false;
        }
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
        // Blue and red share this pose set. isBlue/isRed still control aiming/teleop side.
        PoseFactory degrees = PoseFactory.degrees();
        start = degrees.of(98, 135, 0);
        sample1 = degrees.of(125.5, 135, 5);
        sample1Segment1Start = degrees.of(131.5, 135, 0);
        sample1Segment1End = degrees.of(131.5, 135, 0);
        sample1Segment2Start = degrees.of(131.5, 135, 0);
        sample1Segment2End = degrees.of(131.5, 135, 5);
        sample3Control1 = degrees.of(120.0843, 72.9993, 0);
        sample3 = degrees.of(94.8651, 18.3682, -90);
        sample2Control1 = degrees.of(131.0383, 108.6424, 0);
        sample2Control2 = degrees.of(115.5581, 90.1564, 0);
        sample2 = degrees.of(122.7151, 92.924, 0);
        point2 = degrees.of(60.07, 9.95, -90);
        point4 = degrees.of(94.87, 14.37, -90);
        point5 = degrees.of(95.07, 11.56, -90);
        doneStart = degrees.of(95.07, 9.56, 270);
        done = degrees.of(124.6912, 23.9385, 180);
    }

    private void buildPaths() {
        shootToSample1 = line(start, sample1).heading(Interpolator.piecewise()
                .until(0.8, Interpolator.linear(sample1Segment1Start, sample1Segment1End))
                .until(1, Interpolator.linear(sample1Segment2Start, sample1Segment2End)));
        sample1ToPoint2 = curve(sample1, sample3Control1, sample3).linear(sample1, sample3);
        point2ToSample2 = curve(sample3, sample2Control1, sample2Control2, sample2).linear(sample3, sample2);
        sample2ToShoot2 = line(sample2, point4).linear(sample2, point4);
        point4ToPoint5 = line(point4, point5).constant(point5);
        sample3ToDone = line(doneStart, done).linear(doneStart, done);
    }

    public void gamepadSide() {
        if (gamepad1.y) {
            isBlue = true;
            isRed = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.SHOOTTOSAMPLE1;
        } else if (gamepad1.a) {
            isRed = true;
            isBlue = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.SHOOTTOSAMPLE1;
        }
    }

    private String selectedScenario() {
        if (isBlue) return "BLUE";
        if (isRed) return "RED";
        return "NONE";
    }

    public void runAutoDepo() {
        depo.run_using_pid();

        if (depo.reachedvelocity() && !timers.isRunning("depo")){
            timers.start("depo");
            depo.open_hatch();
            intake.intake_on();
            isHatchClosed = false;
        }
        if (timers.checkSecondsLast("depo", 2)){
            depo.turn_off_deposit();
            depo.close_hatch();
            intake.intake_off();
            isHatchClosed = true;
        }
    }

    private void turnOnShooter() {
        depo.turn_on_deposit();
    }

    public static Pose returnToTeleop() {
        return teleop;
    }

    public boolean isShootingOffset() {
        return isShootOffset;
    }
    public static boolean[] whichSide() {
        return new boolean[] {isBlue, isRed};
    }


    private void updateTelemetry() {
        telemetry.addData("auto state", autoState);
        Pose pose = follower.getPose();
        if (pose != null) {
            telemetry.addData("x", pose.x());
            telemetry.addData("y", pose.y());
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
        }
        telemetry.addData("path time", pathTimer.seconds());
        telemetry.addData("Tag filter", tagAlliance == null ? "ALL" : tagAlliance.name());
        telemetry.addData("Last accepted tag", seenTag == null ? "none" : seenTag.id);
        telemetry.addData("Selected hive", selectedHive == null ? "UNKNOWN" : selectedHive.displayName());
        telemetry.addData("Opposite hive", oppositeHive == null ? "UNRESOLVED" : oppositeHive.displayName());
    }
}