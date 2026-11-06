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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Transfer;
// TURRET DISABLED: import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Bottom_Auto", group = "Auto")
public class Bottom_Auto_Aditi extends OpMode {

    // ---------------- CONFIG ----------------
    private static final String CAMERA_NAME = "Webcam 1";        // must match robot config
    private static final double PARK_TIME_SECONDS = 27.0;        //if no tag is seen
    private static final double SHOT_FEED_SECONDS = 2.0;
    private static final double FLOWER_INTAKE_SECONDS = 3.0;     // min time to intake at the flower
    // TURRET DISABLED: private static final double TURRET_TOLERANCE_DEG = 3.0; // how close the turret must be before shooting
    private static final boolean PRELOAD_SHOT_OPPOSITE = false;
    private static final boolean FLOWER_SHOT_OPPOSITE  = true;
    private static final double PARK_START_HEADING_DEG = -90;    // park leg start heading (change to 90 if it snaps)

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
    private final Pose start      = pose(64.084, 133.1202, -90);
    private final Pose leaveStart = pose(64.141, 108.3963, 90);
    private final Pose gotoFlower = pose(47.0764, 108.512, 90);
    private final Pose flower     = pose(46.8257, 128.0305, 90);
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
    private enum ActionState { IDLE, SHOOT, FLOWER_INTAKE }

    private PathState pathState = PathState.WAIT_FOR_TAG;
    private ActionState actionState = ActionState.IDLE;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime actionTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();
    private boolean shotStarted = false;     // depo reached speed and the hatch is open
    private boolean shootOpposite = false;   // which turret aim the current shot uses (unused while turret is disabled)

    private Side side = null;                // null until a toggle is pressed
    private boolean tagSeen = false;
    private int seenTagId = -1;

    private Follower follower;
    private Depo depo;
    private Intake intake;
    private Transfer transfer;
    // TURRET DISABLED: private Turret turret;
    private AprilTagProcessor tagProcessor;
    private VisionPortal portal;

    // ---------------- OPMODE ----------------
    @Override
    public void init() {
        follower = Robot1.createFollower(hardwareMap);
        depo = new Depo(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        // TURRET DISABLED: turret = new Turret(hardwareMap);

        tagProcessor = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .addProcessor(tagProcessor)
                .build();
    }

    @Override
    public void init_loop() {
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
        telemetry.addLine(side == null ? "NOT READY" : "READY: press START");
        telemetry.update();
    }

    @Override
    public void start() {
        autoTimer.reset();
        follower.setPose(start);
        intake.start();
        depo.start();
        transfer.start();
        // TURRET DISABLED: turret.start();
        // TURRET DISABLED: turret.holdCurrentPosition();
        setActionState(ActionState.IDLE);    // nothing moves till the aprtag is seen
    }

    @Override
    public void loop() {
        if (side == null) {
            telemetry.addLine("pick an alliance.");
            telemetry.update();
            return;
        }

        follower.update();
        // TURRET DISABLED: turret.run_using_pid();
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
        telemetry.addData("Depo velocity", depo.getVelocity());
        // TURRET DISABLED: telemetry.addData("Turret deg / target", "%.1f / %.1f", turret.getPositionDeg(), Turret.deg);
        telemetry.addData("Pose", follower.pose());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterOff();
        if (portal != null) portal.close();
    }

    // ---------------- PATH STATE MACHINE ----------------
    private void updatePathState() {
        switch (pathState) {
            case WAIT_FOR_TAG:                                   // wait till tag is seen
                if (tagSeen) {
                    shootOpposite = PRELOAD_SHOT_OPPOSITE;
                    setActionState(ActionState.SHOOT);           // dshoot preload
                    pathState = PathState.SHOOT_PRELOAD;
                }
                break;

            case SHOOT_PRELOAD:
                if (actionState == ActionState.IDLE) {           // finsihed shootign
                    follower.follow(drive(start, leaveStart));
                    pathState = PathState.LEAVE_START;
                }
                break;

            case LEAVE_START:
                if (!follower.isBusy()) {
                    follower.follow(drive(leaveStart, gotoFlower));
                    pathState = PathState.GO_TO_FLOWER;
                }
                break;

            case GO_TO_FLOWER:
                if (!follower.isBusy()) {
                    follower.follow(drive(gotoFlower, flower));
                    setActionState(ActionState.FLOWER_INTAKE);
                    pathState = PathState.FLOWER;
                }
                break;

            case FLOWER:
                if (!follower.isBusy() && actionTimer.seconds() >= FLOWER_INTAKE_SECONDS) {
                    follower.follow(drive(flower, shootPose));
                    shootOpposite = FLOWER_SHOT_OPPOSITE;
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
    /** Shoots once the robot has stopped and the depo is at speed; ends the shot after SHOT_FEED_SECONDS. */
    private void updateActionState() {
        if (actionState != ActionState.SHOOT) return;
        // TURRET DISABLED: aimTurret();
        // TURRET DISABLED: was  if (!follower.isBusy() && (turretOnTarget() || shotStarted))
        if (follower.isBusy()) return;

        if (!shotStarted && depo.reachedvelocity()) {
            shotStarted = true;
            shotTimer.reset();
            depo.open_hatch();
        } else if (shotStarted && shotTimer.seconds() >= SHOT_FEED_SECONDS) {
            setActionState(ActionState.IDLE);                    // depo, hatch and intake off
        }
    }

    /** Runs the one-time entry actions. The intake only ever turns on together with the depo. */
    private void setActionState(ActionState newState) {
        actionState = newState;
        actionTimer.reset();

        switch (newState) {
            case IDLE:
                shooterOff();
                intake.flower_up();
                // TURRET DISABLED: turret.holdCurrentPosition();
                break;
            case SHOOT:
                shotStarted = false;
                intake.flower_up();
                shooterOn();
                break;
            case FLOWER_INTAKE:
                intake.flower_down();
                shooterOn();
                break;
        }
    }

    private void shooterOn() {
        depo.turn_on_deposit();
        intake.intake_on();
    }

    private void shooterOff() {
        depo.turn_off_deposit();
        depo.close_hatch();
        intake.intake_off();
    }

    // TURRET DISABLED: aimTurret() and turretOnTarget() are commented out until the turret is wired back in.
    // /** Aims the turret at the hive from the robot's live pose, using this side's garden/opposite method. */
    // private void aimTurret() {
    //     Pose p = follower.pose();
    //     if (side == Side.RED) {
    //         if (shootOpposite) turret.turretAngleRedOpposite(p);
    //         else               turret.turretAngleRedGarden(p);
    //     } else {
    //         if (shootOpposite) turret.turretAngleBlueOpposite(p);
    //         else               turret.turretAngleBlueGarden(p);
    //     }
    // }

    // /** Turret.deg holds the last requested aim; wrap it the same way Turret.setPositionDeg does. */
    // private boolean turretOnTarget() {
    //     double target = Turret.deg;
    //     while (target < -90) target += 360;
    //     while (target >= 270) target -= 360;
    //     return Math.abs(turret.getPositionDeg() - target) <= TURRET_TOLERANCE_DEG;
    // }

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