package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.turretLM0.turretAngleBlueGarden;
import static org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.turretLM0.turretAngleBlueOpposite;
import static org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.turretLM0.turretAngleRedGarden;
import static org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.turretLM0.turretAngleRedOpposite;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Robot0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.DepoLM1;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.intakeLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.turretLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Bottom_Auto", group = "Auto")
public class Bottom_Auto_Aditi extends OpMode {

    // =====================================================================
    //  CONFIG
    // =====================================================================
    private static final String CAMERA_NAME = "Webcam 1";    // must match robot config
    private static final double PARK_TIME_SECONDS = 27.0;    // failsafe: go straight to park

    private static final int RED_MIN = 30,  RED_MAX = 33;    // red side tags
    private static final int BLUE_MIN = 38, BLUE_MAX = 41;   // blue side tags

    // --- shooting (TODO tune) ---
    private static final double SHOT_FEED_SECONDS     = 2.0; // hatch open + intake feeding time per shot
    private static final double TURRET_TOLERANCE_DEG  = 3.0; // how close the turret must be before firing
    private static final double FLOWER_INTAKE_SECONDS = 3.0; // flower down + intake time at the flower

    // Which turret aim each shot uses (garden vs opposite). TODO verify for your field.
    private static final boolean PRELOAD_SHOT_OPPOSITE = false;
    private static final boolean FLOWER_SHOT_OPPOSITE  = true;

    // Red park leg: your export starts this leg at heading -90 (parkStart).
    // If the robot's heading snaps at the start of the park leg, change this to shootPose.heading().
    private static final double RED_PARK_START_HEADING_DEG = -90;

    // =====================================================================
    //  POSES  (Pedro Visualizer, heading in degrees)
    //  Filled in by buildRedPoses() / buildBluePoses() for whichever side is selected.
    // =====================================================================
    private static Pose pose(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }
    private Pose start, leaveStart, gotoFlower, flower, shootPose, park;

    /** RIGHT toggle = RED paths. */
    private void buildRedPoses() {
        start       = pose(64.084, 133.1202, -90);
        leaveStart  = pose(64.141, 108.3963, 90);
        gotoFlower  = pose(47.1237, 107.9908, 90);
        flower      = pose(47.2164, 132.5977, 90);
        shootPose   = pose(46.9097, 119.3186, 0);
        park        = pose(7.8308, 114.1196, 0);
    }

    /** LEFT toggle = BLUE paths. (path4 = shoot pose, path5 = park) */
    private void buildBluePoses() {
        start       = pose(76.6039, 7.697, 90);
        leaveStart  = pose(76.6039, 32.667, 270);
        gotoFlower  = pose(93.9336, 32.7901, 270);
        flower      = pose(93.8448, 9.7366, 270);
        shootPose   = pose(94.0808, 20.708, 105);
        park        = pose(132.2665, 25.5141, 0);
    }

    // =====================================================================
    //  PATH STATE
    // =====================================================================
    private enum PathState {
        SHOOT_PRELOAD,   // START: shoot immediately from the start pose
        WAIT_FOR_TAG,    // sit still until the correct AprilTag has been seen
        LEAVE_START,     // start -> leavestart
        GO_TO_FLOWER,    // leavestart -> gotoflower
        FLOWER,          // gotoflower -> flower   (flower intake runs)
        SHOOT_FLOWER,    // flower -> shoot pose   (depo spins while driving, fires on arrival)
        PARK,            // shoot pose -> park (or direct park from failsafe)
        DONE
    }
    private PathState pathState = PathState.SHOOT_PRELOAD;
    private final ElapsedTime pathTimer = new ElapsedTime();

    // =====================================================================
    //  ACTION STATE
    // =====================================================================
    private enum ActionState {
        IDLE,           // everything off
        SHOOT,          // aim turret, spin depo, fire when ready, then back to IDLE
        FLOWER_INTAKE,  // flower down + intake on for FLOWER_INTAKE_SECONDS, then back to IDLE
        STOPPED         // everything off (used by the 27s failsafe)
    }
    private ActionState actionState = ActionState.IDLE;
    private final ElapsedTime actionTimer = new ElapsedTime();

    // =====================================================================
    //  OTHER STATE / HARDWARE
    // =====================================================================
    private enum Side { RED, BLUE }
    private Side side = null;            // null until a toggle is pressed
    private Side appliedSide = null;     // side the poses/paths were last built for
    private boolean tagSeen = false;     // latches true once, first time the right tag is seen
    private int seenTagId = -1;
    private boolean forcedPark = false;

    // shooting bookkeeping
    private boolean shootOpposite = false;   // which turret aim the current shot uses
    private boolean shotFinished = false;    // set when the current shot is done
    private double turretTargetDeg = 0;

    private Follower follower;
    private Path leaveStartPath, gotoFlowerPath, flowerPath, shootFlowerPath, parkPath;

    private DepoLM1 depo;
    private intakeLM0 intake;
    private turretLM0 turret;
    private Timer timers;

    private AprilTagProcessor tagProcessor;
    private VisionPortal portal;
    private final ElapsedTime autoTimer = new ElapsedTime();

    // =====================================================================
    //  OPMODE LIFECYCLE
    // =====================================================================
    @Override
    public void init() {
        follower = Robot0.createFollower(hardwareMap);

        depo = new DepoLM1(hardwareMap);
        intake = new intakeLM0(hardwareMap);
        turret = new turretLM0(hardwareMap);
        timers = new Timer();

        tagProcessor = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .addProcessor(tagProcessor)
                .build();

        telemetry.addLine(sideLabel());
        telemetry.addLine("STEP 1: press RIGHT toggle (RED) or LEFT toggle (BLUE)");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // STEP 1: choose alliance
        if (gamepad1.right_bumper || gamepad1.right_trigger > 0.5) side = Side.RED;
        if (gamepad1.left_bumper  || gamepad1.left_trigger  > 0.5) side = Side.BLUE;

        // STEP 2: load the matching side's paths
        if (side != null && side != appliedSide) applySide(side);
        if (side != null) checkForTag();                            // camera looks for that side's tags

        telemetry.addLine(sideLabel());
        telemetry.addData("STEP 1 - alliance", side == null ? "NOT CHOSEN" : side + " chosen");
        telemetry.addData("STEP 2 - paths loaded", appliedSide == null ? "none" : appliedSide + " paths");
        telemetry.addData("Tag seen", tagSeen ? "YES (id " + seenTagId + ")" : "no - waiting");
        telemetry.addLine(side == null ? "NOT READY" : "STEP 3: press START - robot shoots immediately");
        telemetry.update();
    }

    @Override
    public void start() {
        autoTimer.reset();
        intake.start();
        depo.start();
        turret.holdCurrentPosition();
        timers.createNew("shot");

        // STEP 3: start and immediately shoot the preload from the start pose
        shootOpposite = PRELOAD_SHOT_OPPOSITE;
        setActionState(ActionState.SHOOT);
        setPathState(PathState.SHOOT_PRELOAD);
    }

    @Override
    public void loop() {
        // No side picked = no paths. Stay still.
        if (side == null) {
            telemetry.addLine(sideLabel());
            telemetry.addLine("NOT MOVING. Stop, go back to init, and pick a side.");
            telemetry.update();
            return;
        }

        follower.update();
        turret.run_using_pid();      // PID loops must run every cycle
        depo.run_using_pid();

        if (!tagSeen) checkForTag();
        checkFailsafe();

        updatePathState();
        updateActionState();

        sendTelemetry();
    }

    @Override
    public void stop() {
        depo.turn_off_deposit();
        intake.turn_off_intake();
        if (portal != null) portal.close();
    }

    // =====================================================================
    //  PATH STATE MACHINE  (identical flow for red and blue)
    // =====================================================================
    private void updatePathState() {
        switch (pathState) {

            case SHOOT_PRELOAD:
                if (actionState == ActionState.IDLE) {           // preload shot finished
                    setPathState(PathState.WAIT_FOR_TAG);
                }
                break;

            case WAIT_FOR_TAG:
                if (tagSeen) {                                   // passes straight through if already seen
                    follower.follow(leaveStartPath);
                    setPathState(PathState.LEAVE_START);
                }
                break;

            case LEAVE_START:
                if (!follower.isBusy()) {
                    follower.follow(gotoFlowerPath);
                    setPathState(PathState.GO_TO_FLOWER);
                }
                break;

            case GO_TO_FLOWER:
                if (!follower.isBusy()) {
                    follower.follow(flowerPath);
                    setActionState(ActionState.FLOWER_INTAKE);   // flower down + intake while driving to the flower
                    setPathState(PathState.FLOWER);
                }
                break;

            case FLOWER:
                // wait for BOTH the path and the flower intake to finish
                if (!follower.isBusy() && actionState == ActionState.IDLE) {
                    follower.follow(shootFlowerPath);
                    shootOpposite = FLOWER_SHOT_OPPOSITE;
                    setActionState(ActionState.SHOOT);           // depo spins while driving, fires on arrival
                    setPathState(PathState.SHOOT_FLOWER);
                }
                break;

            case SHOOT_FLOWER:
                if (actionState == ActionState.IDLE) {           // shot finished
                    follower.follow(parkPath);
                    setPathState(PathState.PARK);
                }
                break;

            case PARK:
                if (!follower.isBusy()) setPathState(PathState.DONE);
                break;

            case DONE:
                break;
        }
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.reset();
    }

    // =====================================================================
    //  ACTION STATE MACHINE  (identical for red and blue)
    // =====================================================================
    private void updateActionState() {
        switch (actionState) {

            case IDLE:
                break;

            case SHOOT:
                aimTurret();                                     // keep aiming from the live pose
                // fire only when the robot has stopped AND the turret is on target
                // (once the hatch has opened, keep going even if the turret drifts)
                if (!follower.isBusy() && (turretOnTarget() || timers.isRunning("shot"))) {
                    runShot();
                }
                if (shotFinished) {
                    setActionState(ActionState.IDLE);
                }
                break;

            case FLOWER_INTAKE:
                if (actionTimer.seconds() >= FLOWER_INTAKE_SECONDS) {
                    setActionState(ActionState.IDLE);            // entry of IDLE: flower up, intake off
                }
                break;

            case STOPPED:
                break;
        }
    }

    /** Switches action state and runs the ONE-TIME entry actions for the new state. */
    private void setActionState(ActionState newState) {
        actionState = newState;
        actionTimer.reset();

        switch (newState) {
            case IDLE:
                intake.turn_off_intake();
                intake.flower_up();
                break;

            case STOPPED:
                timers.stop("shot");
                depo.turn_off_deposit();
                depo.close_hatch();
                intake.turn_off_intake();
                intake.flower_up();
                break;

            case SHOOT:
                shotFinished = false;
                depo.turn_on_deposit();
                // Standing still (the preload shot): intake and depo start at the same time.
                // Still driving (the flower shot): depo spins up now, intake joins when the hatch opens.
                if (!follower.isBusy()) intake.turn_on_intake();
                break;

            case FLOWER_INTAKE:
                intake.flower_down();
                intake.turn_on_intake();
                break;
        }
    }

    // =====================================================================
    //  SHOOTING
    // =====================================================================
    /** Wait for depo speed, open the hatch and feed for SHOT_FEED_SECONDS, then shut everything off. */
    private void runShot() {
        if (depo.reachedvelocity() && !timers.isRunning("shot")) {
            timers.start("shot");
            depo.open_hatch();
            intake.turn_on_intake();
        }
        if (timers.checkSecondsLast("shot", SHOT_FEED_SECONDS)) {
            depo.turn_off_deposit();
            depo.close_hatch();
            intake.turn_off_intake();
            shotFinished = true;
        }
    }

    /** Aims the turret at the hive from the robot's live pose, using this side's garden/opposite function. */
    private void aimTurret() {
        Pose p = follower.pose();
        double x = p.x();
        double y = p.y();
        double headingDeg = Math.toDegrees(p.heading());

        double deg;
        if (side == Side.RED) {
            deg = shootOpposite ? turretAngleRedOpposite(x, y, headingDeg)
                    : turretAngleRedGarden(x, y, headingDeg);
        } else {
            deg = shootOpposite ? turretAngleBlueOpposite(x, y, headingDeg)
                    : turretAngleBlueGarden(x, y, headingDeg);
        }
        if (deg < -90) deg += 360;

        turretTargetDeg = deg;
        turret.setPositionDeg(deg);
    }

    private boolean turretOnTarget() {
        return Math.abs(turret.getPositionDeg() - turretTargetDeg) <= TURRET_TOLERANCE_DEG;
    }

    // =====================================================================
    //  27s FAILSAFE
    // =====================================================================
    private void checkFailsafe() {
        boolean alreadyParking = pathState == PathState.PARK || pathState == PathState.DONE;
        if (autoTimer.seconds() >= PARK_TIME_SECONDS && !forcedPark && !alreadyParking) {
            forcedPark = true;

            follower.stop();                           // cancel whatever is running
            setActionState(ActionState.STOPPED);       // depo off, hatch closed, intake off, flower up

            Pose here = follower.pose();
            Path direct = line(here, park).linear(here.heading(), park.heading());

            follower.follow(direct);                   // straight to THIS side's park pose
            setPathState(PathState.PARK);              // no other paths run after this
        }
    }

    // =====================================================================
    //  APRILTAG
    // =====================================================================
    private void checkForTag() {
        if (tagSeen || side == null) return;
        int min = (side == Side.RED) ? RED_MIN : BLUE_MIN;
        int max = (side == Side.RED) ? RED_MAX : BLUE_MAX;

        for (AprilTagDetection d : tagProcessor.getDetections()) {
            if (d.id >= min && d.id <= max) {
                tagSeen = true;
                seenTagId = d.id;
                break;
            }
        }
    }

    // =====================================================================
    //  SIDE SELECT / PATH BUILDING  (from your Visualizer exports)
    // =====================================================================
    private void applySide(Side s) {
        appliedSide = s;

        if (s == Side.RED) buildRedPoses();
        else               buildBluePoses();

        buildPaths(s);
        follower.setPose(start);

        // a tag latched for the OTHER side must not start this side's paths
        tagSeen = false;
        seenTagId = -1;
    }

    private void buildPaths(Side s) {
        leaveStartPath  = line(start, leaveStart).linear(start.heading(), leaveStart.heading());
        gotoFlowerPath  = line(leaveStart, gotoFlower).linear(leaveStart.heading(), gotoFlower.heading());
        flowerPath      = line(gotoFlower, flower).linear(gotoFlower.heading(), flower.heading());
        shootFlowerPath = line(flower, shootPose).linear(flower.heading(), shootPose.heading());

        if (s == Side.RED) {
            // red export: linear heading from parkStart (-90) to park (0)
            parkPath = line(shootPose, park).linear(Math.toRadians(RED_PARK_START_HEADING_DEG), park.heading());
        } else {
            // blue export: linear heading shoot pose -> park
            parkPath = line(shootPose, park).linear(shootPose.heading(), park.heading());
        }
    }

    // =====================================================================
    //  TELEMETRY
    // =====================================================================
    private String sideLabel() {
        if (side == Side.RED)  return ">>> RED SIDE (right toggle) | tags 30-33 <<<";
        if (side == Side.BLUE) return ">>> BLUE SIDE (left toggle) | tags 38-41 <<<";
        return "!!! NO SIDE SELECTED - press RIGHT (red) or LEFT (blue) !!!";
    }

    private void sendTelemetry() {
        telemetry.addLine(sideLabel());                 // always the first line during the match
        telemetry.addData("Tag seen", tagSeen ? "YES (id " + seenTagId + ")" : "no - waiting");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Action State", actionState);
        telemetry.addData("Auto Timer", "%.1f / %.0f", autoTimer.seconds(), PARK_TIME_SECONDS);
        telemetry.addData("Forced Park", forcedPark);
        telemetry.addData("Depo velocity", depo.getVelocity());
        telemetry.addData("Turret deg / target", "%.1f / %.1f", turret.getPositionDeg(), turretTargetDeg);
        telemetry.addData("Pose", follower.pose());
        telemetry.update();
    }
}