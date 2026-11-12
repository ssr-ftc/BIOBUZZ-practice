package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Depo;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Bottom_Auto", group = "Auto")
public class Bottom_Auto_Aditi extends OpMode {

    // ---------------- CONFIG ----------------
    private static final String CAMERA_NAME = "Webcam 1";        // must match robot config
    private static final double PARK_TIME_SECONDS = 27.0;        // if no tag is seen
    private static final double FLOWER_INTAKE_SECONDS = 3.0;     // min time to intake at the flower
    private static final double PARK_START_HEADING_DEG = -90;    // park leg start heading (change to 90 if it snaps)

    // --- shooting (same as EntireAuto) ---
    /** Open the hatch (and start the intake) this long after the flywheel is at speed. */
    private static final double SHOT_OPEN_SECONDS = 0.3;
    /** How long every shot runs once the flywheel is at speed. */
    private static final double SHOOT_HOLD_SECONDS = 2;

    // --- shot velocity ---
    /** Depo's own ON_VELOCITY (Depo.java). Update this if you change it there. */
    private static final double DEPO_BASE_VELOCITY = 1520;
    /** Extra velocity added to every shot. */
    private static final double SHOT_VELOCITY_BOOST = 0; //can be used if needed
    private static final double SHOT_VELOCITY = DEPO_BASE_VELOCITY + SHOT_VELOCITY_BOOST;

    // --- turret angles (fixed, no hive auto-aim) ---
    private static final double TURRET_START_DEG = -180;         // from init, through the preload shot and the flower trip
    private static final double TURRET_SHOOT_DEG = -195;         // set when we leave the flower for the shootPose

    /** Right toggle = RED (tags 30-33), left toggle = BLUE (tags 38-41). Both use the same poses. */
    private enum Side {
        RED(30, 33), BLUE(38, 41);
        final int minTag, maxTag;
        Side(int minTag, int maxTag) { this.minTag = minTag; this.maxTag = maxTag; }
    }

    // ---------------- POSES (Pedro Visualizer, degrees) ----------------
    private static Pose pose(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }
    private final Pose start      = pose(64.084, 133.1202, 90);
    private final Pose leaveStart = pose(64.141, 117.5, 90);
    private final Pose gotoFlower = pose(46.0764, 117.5, 90);
    private final Pose flower     = pose(46.8257, 127, 90);
    private final Pose shootPose  = pose(46.9097, 119.2908, 90);
    private final Pose park       = pose(7.8308, 114.1196, 0);

    private static Path drive(Pose from, Pose to) {
        return line(from, to).linear(from.heading(), to.heading());
    }

    // ---------------- STATE ----------------
    private enum PathState {
        WAIT_FOR_TAG, SHOOT_PRELOAD, LEAVE_START, GO_TO_FLOWER, FLOWER, SHOOT_FLOWER,
        FAILSAFE_LEAVE, PARK, DONE
    }
    private enum ActionState { IDLE, SHOOT, APPROACH_INTAKE, FLOWER_INTAKE }

    private PathState pathState = PathState.WAIT_FOR_TAG;
    private ActionState actionState = ActionState.IDLE;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime actionTimer = new ElapsedTime();

    private Side side = null;                // null until a toggle is pressed
    private boolean tagSeen = false;
    private int seenTagId = -1;

    private Follower follower;
    private Depo depo;
    private Intake intake;
    private Turret turret;
    private Timer timers;
    private AprilTagProcessor tagProcessor;
    private VisionPortal portal;

    // ---------------- OP MODE ----------------
    @Override
    public void init() {
        follower = Robot1.createFollower(hardwareMap);
        depo = new Depo(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        timers = new Timer();
        timers.createNew("shoot");

        turret.resetTurretEncoders();                    // zero the encoder here (turret must be at its home position on init)
        turret.setPositionDeg(TURRET_START_DEG);         // face the start angle from init

        tagProcessor = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .addProcessor(tagProcessor)
                .build();
    }

    @Override
    public void init_loop() {
        turret.start();                                  // refresh PIDF every loop (so Dashboard tuning applies)
        turret.run_using_pid();                          // hold the start angle during init

        Side picked = side;
        if (gamepad1.right_bumper || gamepad1.right_trigger > 0.5) picked = Side.RED;
        if (gamepad1.left_bumper  || gamepad1.left_trigger  > 0.5) picked = Side.BLUE;
        if (picked != side) {                // new alliance = new tag range, forget any old tag
            side = picked;
            tagSeen = false;
            seenTagId = -1;
        }

        telemetry.addData("Alliance", allianceLabel());
        telemetry.addData("APRILTAG", tagSeen ? "SEEN (id " + seenTagId + ")" : "NOT SEEN");
        telemetry.addData("Visible tag IDs", visibleTagIds());
        telemetry.addData("Turret deg / target", "%.1f / %.1f", turret.getPositionDeg(), turret.getTargetDeg());
        telemetry.addLine(side == null ? "NOT READY" : "READY: press START");
        telemetry.update();
    }

    @Override
    public void start() {
        autoTimer.reset();
        follower.setPose(start);
        intake.start();
        depo.start();
        turret.setPositionDeg(TURRET_START_DEG);         // set the target again at start
        turret.start();
        setActionState(ActionState.IDLE);    // nothing moves till the AprilTag is seen
    }

    @Override
    public void loop() {
        if (side == null) {
            telemetry.addLine("pick an alliance.");
            telemetry.update();
            return;
        }

        follower.update();
        turret.run_using_pid();
        depo.run_using_pid();
        checkForTag();
        updatePathState();
        updateActionState();
        checkFailsafe();

        telemetry.addData("Alliance", allianceLabel());
        telemetry.addData("APRILTAG", tagSeen ? "SEEN (id " + seenTagId + ")" : "NOT SEEN");
        telemetry.addData("Visible tag IDs", visibleTagIds());
        telemetry.addData("Path / Action", pathState + " / " + actionState);
        telemetry.addData("Auto timer", "%.1f / %.0f", autoTimer.seconds(), PARK_TIME_SECONDS);
        telemetry.addData("Depo velocity / target", "%.0f / %.0f", depo.getVelocity(), SHOT_VELOCITY);
        telemetry.addData("Turret deg / target", "%.1f / %.1f", turret.getPositionDeg(), turret.getTargetDeg());
        telemetry.addData("Pose", follower.pose());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterOff();
        turret.turretOff();
        if (portal != null) portal.close();
    }

    // ---------------- PATH STATE MACHINE ----------------
    private void updatePathState() {
        switch (pathState) {
            case WAIT_FOR_TAG:                                   // wait till tag is seen
                if (tagSeen) {
                    setActionState(ActionState.SHOOT);           // shoot preload
                    pathState = PathState.SHOOT_PRELOAD;
                }
                break;

            case SHOOT_PRELOAD:
                if (actionState == ActionState.IDLE) {           // finished shooting
                    follower.follow(drive(start, leaveStart));
                    pathState = PathState.LEAVE_START;
                }
                break;

            case LEAVE_START:
                if (!follower.isBusy()) {
                    follower.follow(drive(leaveStart, gotoFlower));
                    setActionState(ActionState.APPROACH_INTAKE); // all intakes on and down while driving to the flower
                    pathState = PathState.GO_TO_FLOWER;
                }
                break;

            case GO_TO_FLOWER:
                if (!follower.isBusy()) {
                    follower.follow(drive(gotoFlower, flower));
                    setActionState(ActionState.FLOWER_INTAKE);   // front + back intakes stay down while entering the flower
                    pathState = PathState.FLOWER;
                }
                break;

            case FLOWER:
                if (!follower.isBusy() && actionTimer.seconds() >= FLOWER_INTAKE_SECONDS) {
                    follower.follow(drive(flower, shootPose));
                    turret.setPositionDeg(TURRET_SHOOT_DEG);     // start angle -> shoot angle while driving to the shootPose
                    setActionState(ActionState.SHOOT);
                    pathState = PathState.SHOOT_FLOWER;
                }
                break;

            case SHOOT_FLOWER:
                if (actionState == ActionState.IDLE) {           // shooting done
                    follower.follow(line(shootPose, park)
                            .linear(Math.toRadians(PARK_START_HEADING_DEG), park.heading()));
                    pathState = PathState.PARK;
                }
                break;

            case FAILSAFE_LEAVE:                                 // cancel everything and park
                if (!follower.isBusy()) {
                    follower.follow(drive(leaveStart, park));
                    pathState = PathState.PARK;
                }
                break;

            case PARK:
                if (!follower.isBusy()) pathState = PathState.DONE;
                break;

            case DONE:
                break;
        }
    }

    // ---------------- ACTIONS ----------------
    /** While shooting: wait until the robot has stopped, then run the shot. Back to IDLE when it finishes. */
    private void updateActionState() {
        if (actionState != ActionState.SHOOT) return;
        if (follower.isBusy()) return;                           // depo keeps spinning while we drive in

        if (finishShot()) {
            setActionState(ActionState.IDLE);
        }
    }

    /** Runs the one-time entry actions. */
    private void setActionState(ActionState newState) {
        actionState = newState;
        actionTimer.reset();

        switch (newState) {
            case IDLE:
                timers.stop("shoot");
                shooterOff();
                intake.flower_up();
                intake.front_up();                               // retract everything (also covers the failsafe)
                intake.back_up();
                break;
            case SHOOT:
                timers.stop("shoot");                            // fresh shot
                intake.flower_up();
                spinUpShooter();                                 // pre-spin; the intake starts when the hatch opens
                break;
            case APPROACH_INTAKE:
                intake.flower_down();                            // every intake on while driving to the flower:
                intake.front_down();                             // flower, front, back, and both rollers
                intake.back_down();
                intake.intake_on();
                break;
            case FLOWER_INTAKE:
                intake.flower_down();
                intake.front_down();                             // front servo stays at frontDownPos
                intake.back_down();                              // back servo stays at backDownPos
                spinUpShooter();                                 // pre-spin for the flower shot while intaking
                intake.intake_on();
                break;
        }
    }

    /**
     * One shot (same logic as EntireAuto). True on the loop it finishes and the flywheel is turned off.
     * The shoot timer starts once the flywheel is at speed. The intake and hatch open SHOT_OPEN_SECONDS later,
     * and the shot ends SHOOT_HOLD_SECONDS after the flywheel reached speed.
     */
    private boolean finishShot() {
        if (!depo.isDepositOn() && !timers.isRunning("shoot")) {
            spinUpShooter();
        }
        if (depo.reachedvelocity() && !timers.isRunning("shoot")) {
            timers.start("shoot");
        }
        if (timers.checkSeconds("shoot", SHOT_OPEN_SECONDS)) {
            intake.intake_on();
            depo.open_hatch();
        }
        if (timers.checkSecondsLast("shoot", SHOOT_HOLD_SECONDS)) {
            shooterOff();                                        // hatch closed, depo off, intake off
            return true;
        }
        return false;
    }

    /** Turns the flywheel on at the shot velocity (Depo's default + SHOT_VELOCITY_BOOST). */
    private void spinUpShooter() {
        depo.turn_on_deposit();                                  // resets the PID and turns the depo on
        depo.set_target_velocity(SHOT_VELOCITY);                 // then raise the target
    }

    private void shooterOff() {
        depo.close_hatch();
        depo.turn_off_deposit();
        intake.intake_off();
    }

    // ---------------- 27s FAILSAFE: leave start, then park ----------------
    private void checkFailsafe() {
        boolean alreadyDone = pathState == PathState.FAILSAFE_LEAVE
                || pathState == PathState.PARK || pathState == PathState.DONE;
        if (autoTimer.seconds() < PARK_TIME_SECONDS || alreadyDone) return;

        setActionState(ActionState.IDLE);                        // everything off
        follower.follow(drive(follower.pose(), leaveStart));     // leg 1: wherever we are -> leave start
        pathState = PathState.FAILSAFE_LEAVE;                    // leg 2 (-> park) runs in FAILSAFE_LEAVE
    }

    // ---------------- APRILTAG ----------------
    private void checkForTag() {
        if (tagSeen || side == null) return;
        for (AprilTagDetection d : tagProcessor.getDetections()) {
            if (d.id >= side.minTag && d.id <= side.maxTag) {
                tagSeen = true;
                seenTagId = d.id;
                return;
            }
        }
    }

    private String visibleTagIds() {
        StringBuilder sb = new StringBuilder();
        for (AprilTagDetection d : tagProcessor.getDetections()) sb.append(d.id).append(' ');
        return sb.length() == 0 ? "none" : sb.toString().trim();
    }

    private String allianceLabel() {
        return side == null ? "NONE - press RIGHT (red) or LEFT (blue)"
                : side + " (tags " + side.minTag + "-" + side.maxTag + ")";
    }
}