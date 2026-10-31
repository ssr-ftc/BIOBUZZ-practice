package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.CustomFollower;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.DepoLM1;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0.intakeLM0;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Bottom_Auto", group = "Auto")
public class Bottom_Auto_Aditi extends OpMode {

    // =====================================================================
    //  CONFIG
    // =====================================================================
    private static final String CAMERA_NAME = "Webcam 1";   // the WEBCAM's name in the robot config
    private static final double PARK_TIME_SECONDS = 27.0;   // failsafe: go straight to park

    private static final int RED_MIN = 30,  RED_MAX = 33;   // red side tags
    private static final int BLUE_MIN = 38, BLUE_MAX = 41;  // blue side tags

    private static final double INTAKE_TIME_SECONDS    = 1.0;  // TODO tune: flower intake time
    private static final double SPINUP_TIMEOUT_SECONDS = 3.0;  // shoot anyway if velocity never reached
    private static final double SHOOT_TIME_SECONDS     = 1.5;  // TODO tune: hatch open time

    private static final double TURRET_TOLERANCE_DEG   = 3.0;  // TODO tune
    private static final double TURRET_TIMEOUT_SECONDS = 2.0;  // fire anyway if turret never settles

    // =====================================================================
    //  POSES  (Pedro Visualizer, heading in degrees)
    // =====================================================================
    private static Pose pose(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }
    private final Pose start       = pose(64.7738, 133.9711, -90);
    private final Pose gotoFlower  = pose(47.0635, 122.0681, 90);
    private final Pose flower      = pose(47.1237, 130.5576, 90);
    private final Pose shootFlower = pose(47.2164, 122.2292, 290);
    private final Pose park        = pose(7.8753, 114.1343, 0);

    // =====================================================================
    //  PATH STATE
    // =====================================================================
    private enum PathState {
        WAIT_FOR_TAG,    // 1-2. sit still until the correct AprilTag is seen
        SHOOT_PRELOAD,   // 3.   shoot into the hive from the start pose
        GO_TO_FLOWER,    // 4.   start -> gotoflower
        FLOWER,          // 5.   gotoflower -> flower   (flower intake runs)
        SHOOT_FLOWER,    // 6.   flower -> shootflower
        SHOOTING,        // 7.   sit at shootflower while the shot happens
        PARK,            // 8.   shootflower -> park (or direct park from failsafe)
        DONE
    }
    private PathState pathState = PathState.WAIT_FOR_TAG;
    private final ElapsedTime pathTimer = new ElapsedTime();

    // =====================================================================
    //  ACTION STATE
    // =====================================================================
    private enum ActionState {
        IDLE,       // everything off, hatch closed, flower up
        INTAKE,     // flower down + intake on for INTAKE_TIME_SECONDS
        SPIN_UP,    // intake + depo start together; waiting for velocity + turret on target
        SHOOT,      // hatch open, intake still feeding, for SHOOT_TIME_SECONDS
        STOPPED     // everything off (used by the 27s failsafe)
    }
    private ActionState actionState = ActionState.IDLE;
    private final ElapsedTime actionTimer = new ElapsedTime();

    // =====================================================================
    //  OTHER STATE / HARDWARE
    // =====================================================================
    private enum Side { RED, BLUE }
    private Side side = null;
    private boolean tagSeen = false;     // latches true once, first time the right tag is seen
    private int seenTagId = -1;
    private boolean forcedPark = false;
    private double turretTargetDeg = 0;

    private CustomFollower follower;
    private PathChain gotoFlowerPath, flowerPath, shootFlowerPath, parkPath;

    private intakeLM0 intake;
    private DepoLM1 depo;
    private turret turretSub;

    private AprilTagProcessor tagProcessor;
    private VisionPortal portal;
    private final ElapsedTime autoTimer = new ElapsedTime();

    // =====================================================================
    //  OPMODE LIFECYCLE
    // =====================================================================
    @Override
    public void init() {
        follower = new CustomFollower(hardwareMap, telemetry);
        follower.setPose(start);
        buildPaths();

        intake = new intakeLM0(hardwareMap);
        depo = new DepoLM1(hardwareMap);
        turretSub = new turret(hardwareMap);
        turretSub.ResetTurretEncoders();     // assumes the turret starts centered, facing forward
        turretSub.setPositionDeg(0);

        tagProcessor = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .addProcessor(tagProcessor)
                .build();

        telemetry.addLine("RIGHT bumper = RED (tags 30-33) | LEFT bumper = BLUE (tags 38-41)");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.right_bumper || gamepad1.right_trigger > 0.5) side = Side.RED;
        if (gamepad1.left_bumper  || gamepad1.left_trigger  > 0.5) side = Side.BLUE;

        if (side != null) checkForTag();   // camera starts looking once a side is chosen

        telemetry.addData("Side", side == null ? "NONE - pick one!" : side);
        telemetry.addData("Tag seen", tagSeen ? "YES (id " + seenTagId + ")" : "no");
        telemetry.update();
    }

    @Override
    public void start() {
        if (side == null) side = Side.RED;   // fallback if nothing was picked
        intake.start();
        depo.start();
        turretSub.start();
        autoTimer.reset();
        setPathState(PathState.WAIT_FOR_TAG);
        setActionState(ActionState.IDLE);
    }

    @Override
    public void loop() {
        follower.update();

        if (!tagSeen) checkForTag();
        checkFailsafe();

        updatePathState();
        updateActionState();

        // PID loops must run every cycle
        depo.run_using_pid();
        turretSub.run_using_pid();

        sendTelemetry();
    }

    @Override
    public void stop() {
        depo.turn_off_deposit();
        intake.turn_off_intake();
        if (portal != null) portal.close();
    }

    // =====================================================================
    //  PATH STATE MACHINE
    // =====================================================================
    private void updatePathState() {
        switch (pathState) {

            case WAIT_FOR_TAG:
                if (tagSeen) {
                    aimTurret();                                 // aim at the hive from the start pose
                    setActionState(ActionState.SPIN_UP);         // intake + depo start together
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (actionState == ActionState.IDLE) {           // preload shot finished
                    follower.followPath(gotoFlowerPath, true);
                    setPathState(PathState.GO_TO_FLOWER);
                }
                break;

            case GO_TO_FLOWER:
                if (!follower.isBusy()) {
                    follower.followPath(flowerPath, true);
                    setActionState(ActionState.INTAKE);          // flower down + intake on
                    setPathState(PathState.FLOWER);
                }
                break;

            case FLOWER:
                // wait for BOTH the path and the flower intake to finish
                if (!follower.isBusy() && actionState == ActionState.IDLE) {
                    follower.followPath(shootFlowerPath, true);
                    aimTurret();                                 // start turning while driving
                    setPathState(PathState.SHOOT_FLOWER);
                }
                break;

            case SHOOT_FLOWER:
                if (!follower.isBusy()) {                        // arrived at the shoot pose
                    aimTurret();                                 // re-aim from the REAL stopped pose
                    setActionState(ActionState.SPIN_UP);         // intake + depo start together
                    setPathState(PathState.SHOOTING);
                }
                break;

            case SHOOTING:
                if (actionState == ActionState.IDLE) {           // shot finished
                    follower.followPath(parkPath, true);
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
    //  ACTION STATE MACHINE
    // =====================================================================
    private void updateActionState() {
        switch (actionState) {

            case IDLE:
                break;

            case INTAKE:
                if (actionTimer.seconds() >= INTAKE_TIME_SECONDS) {
                    setActionState(ActionState.IDLE);            // entry of IDLE: intake off, flower up
                }
                break;

            case SPIN_UP:
                // fire when the deposit is at speed AND the turret is on target
                // (each has a timeout so the auto never gets stuck)
                if ((depo.reachedvelocity() || actionTimer.seconds() >= SPINUP_TIMEOUT_SECONDS)
                        && (turretOnTarget() || actionTimer.seconds() >= TURRET_TIMEOUT_SECONDS)) {
                    setActionState(ActionState.SHOOT);
                }
                break;

            case SHOOT:
                // hatch is open and the intake is still running, so it feeds the shot
                if (actionTimer.seconds() >= SHOOT_TIME_SECONDS) {
                    setActionState(ActionState.IDLE);            // entry of IDLE: hatch closed, depo off, intake off
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
            case STOPPED:
                intake.turn_off_intake();
                intake.flower_up();
                depo.close_hatch();
                depo.turn_off_deposit();
                break;

            case INTAKE:
                intake.flower_down();
                intake.turn_on_intake();
                break;

            case SPIN_UP:
                intake.turn_on_intake();      // intake and depo start at the SAME time
                depo.turn_on_deposit();
                break;

            case SHOOT:
                depo.open_hatch();            // intake is already running from SPIN_UP
                break;
        }
    }

    // =====================================================================
    //  TURRET
    // =====================================================================
    /** Aims the turret at the hive from wherever the robot actually is right now. */
    private void aimTurret() {
        Pose p = follower.getPose();
        turretTargetDeg = turret.turretAngle(p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        turretSub.setPositionDeg(turretTargetDeg);
    }

    private boolean turretOnTarget() {
        return Math.abs(turretSub.getPositionDeg() - turretTargetDeg) <= TURRET_TOLERANCE_DEG;
    }

    // =====================================================================
    //  27s FAILSAFE
    // =====================================================================
    private void checkFailsafe() {
        boolean alreadyParking = pathState == PathState.PARK || pathState == PathState.DONE;
        if (autoTimer.seconds() >= PARK_TIME_SECONDS && !forcedPark && !alreadyParking) {
            forcedPark = true;

            follower.pedro.breakFollowing();           // cancel the current path
            setActionState(ActionState.STOPPED);       // stop intake/deposit, close hatch

            Pose here = follower.getPose();
            PathChain direct = follower.pathBuilder()
                    .addPath(new BezierLine(here, park))
                    .setLinearHeadingInterpolation(here.getHeading(), park.getHeading())
                    .build();

            follower.followPath(direct, true);
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
    //  PATH BUILDING
    // =====================================================================
    private void buildPaths() {
        gotoFlowerPath = follower.pathBuilder()
                .addPath(new BezierLine(start, gotoFlower))
                .setLinearHeadingInterpolation(start.getHeading(), gotoFlower.getHeading())
                .build();

        flowerPath = follower.pathBuilder()
                .addPath(new BezierLine(gotoFlower, flower))
                .setLinearHeadingInterpolation(gotoFlower.getHeading(), flower.getHeading())
                .build();

        shootFlowerPath = follower.pathBuilder()
                .addPath(new BezierLine(flower, shootFlower))
                .setLinearHeadingInterpolation(flower.getHeading(), shootFlower.getHeading())
                .build();

        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(shootFlower, park))
                .setLinearHeadingInterpolation(shootFlower.getHeading(), park.getHeading())
                .build();
    }

    // =====================================================================
    //  TELEMETRY
    // =====================================================================
    private void sendTelemetry() {
        telemetry.addData("Side", side);
        telemetry.addData("Tag seen", tagSeen ? "YES (id " + seenTagId + ")" : "no");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Action State", actionState);
        telemetry.addData("Auto Timer", "%.1f", autoTimer.seconds());
        telemetry.addData("Force Park", forcedPark);
        telemetry.addData("Depo velocity", depo.getVelocity());
        telemetry.addData("Turret deg / target", "%.1f / %.1f", turretSub.getPositionDeg(), turretTargetDeg);
        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
    }
}