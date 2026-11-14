//package org.firstinspires.ftc.teamcode.pedroPathing.Auto;
//
//import com.pedropathing.api.Paths;
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.math.Pose;
//import com.pedropathing.paths.Path;
//
//import static com.pedropathing.api.Paths.line;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Depo;
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer;
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//@Autonomous(name = "Bottom_Auto misha", group = "Auto")
//public class Bottom_Auto_real extends OpMode {
//
//    // ---------------- CONFIG ----------------
//    private static final String CAMERA_NAME = "Webcam 1";        // must match robot config
//    private static final double PARK_TIME_SECONDS = 27.0;        // if no tag is seen
//    private static final double FLOWER_INTAKE_SECONDS = 3.0;     // min time to intake at the flower
//    private static final double PARK_START_HEADING_DEG = -90;    // park leg start heading (change to 90 if it snaps)
//
//    // --- shooting (same as EntireAuto) ---
//    /** Open the hatch (and start the intake) this long after the flywheel is at speed. */
//    private static final double SHOT_OPEN_SECONDS = 0.3;
//    /** How long every shot runs once the flywheel is at speed. */
//    private static final double SHOOT_HOLD_SECONDS = 2;
//
//    // --- shot velocity ---
//    /** Depo's own ON_VELOCITY (Depo.java). Update this if you change it there. */
//    private static final double DEPO_BASE_VELOCITY = 1520;
//    /** Extra velocity added to every shot. */
//    private static final double SHOT_VELOCITY_BOOST = 0; //can be used if needed
//    private static final double SHOT_VELOCITY = DEPO_BASE_VELOCITY + SHOT_VELOCITY_BOOST;
//
//    // --- turret angles (fixed, no hive auto-aim) ---
//    private static final double TURRET_START_DEG = -175;         // from init, through the preload shot and the flower trip
//    private static final double TURRET_SHOOT_DEG = -195;         // set when we leave the flower for the shootPose
//
//    /** Right toggle = RED (tags 30-33), left toggle = BLUE (tags 38-41). Both use the same poses. */
//    private enum Side {
//        RED(30, 33), BLUE(38, 41);
//        final int minTag, maxTag;
//        Side(int minTag, int maxTag) { this.minTag = minTag; this.maxTag = maxTag; }
//    }
//    boolean autoaura=false;
//
//    // ---------------- POSES (Pedro Visualizer, degrees) ----------------
//    private static Pose pose(double x, double y, double headingDeg) {
//        return new Pose(x, y, Math.toRadians(headingDeg));
//    }
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//    private final Pose start      = poseFactory.of(64.084, 133, 90);
//    private final Pose path1 = poseFactory.of(47, 115.2191, 90);
//    private final Pose point2 = poseFactory.of(47, 129.5, 90);
//    private final Pose point3 = poseFactory.of(59.0723, 128.345, 90);
//    private final Pose point3Control1 = poseFactory.of(52.9334, 101.0656, 0);
//    private final Pose point4Start = poseFactory.of(59.0723, 128.345, 90);
//    private final Pose point4 = poseFactory.of(11.2936, 120.5227, 0);
//    private final Pose point4Control1 = poseFactory.of(33.2009, 118.4988, 0);
//
//    public Path path1() {
//        return Paths.line(start, path1).constant(path1);
//    }
//
//    public Path path2() {
//        return Paths.line(path1, point2).constant(point2);
//    }
//
//    public Path path3() {
//        return Paths.curve(point2, point3Control1, point3).constant(point3);
//    }
//
//    public Path path4() {
//        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
//    }
//
//    private static Path drive(Pose from, Pose to) {
//        return line(from, to).linear(from.heading(), to.heading());
//    }
//
//    // ---------------- STATE ----------------
//    private enum PathState {
//        WAIT_FOR_TAG, SHOOT_PRELOAD, LEAVE_START, GO_TO_FLOWER, FLOWER, SHOOT_FLOWER,
//        FAILSAFE_LEAVE, PARK, DONE
//    }
//    private enum ActionState { IDLE, SHOOT, APPROACH_INTAKE, FLOWER_INTAKE }
//
//    private int pathState = 1;
//    private ActionState actionState = ActionState.IDLE;
//    private final ElapsedTime autoTimer = new ElapsedTime();
//    private final ElapsedTime actionTimer = new ElapsedTime();
//
//    private Side side = null;                // null until a toggle is pressed
//    private boolean tagSeen = false;
//    private int seenTagId = -1;
//
//    private Follower follower;
//    private Depo depo;
//    private Intake intake;
//    private Turret turret;
//    private Timer timers;
//    private AprilTagProcessor tagProcessor;
//    private VisionPortal portal;
//    ElapsedTime didytimer;
//
//    // ---------------- OP MODE ----------------
//    @Override
//    public void init() {
//        follower = Robot1.createFollower(hardwareMap);
//        depo = new Depo(hardwareMap);
//        intake = new Intake(hardwareMap);
//        turret = new Turret(hardwareMap);
//        timers = new Timer();
//        didytimer = new ElapsedTime();
//        timers.createNew("shoot");
//
//        turret.resetTurretEncoders();                    // zero the encoder here (turret must be at its home position on init)
//        turret.setPositionDeg(TURRET_START_DEG);         // face the start angle from init
//
//        tagProcessor = new AprilTagProcessor.Builder().build();
//        portal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
//                .addProcessor(tagProcessor)
//                .build();
//    }
//
//    @Override
//    public void init_loop() {
//        turret.start();                                  // refresh PIDF every loop (so Dashboard tuning applies)
//        turret.run_using_pid();                          // hold the start angle during init
//
//        Side picked = side;
//        if (gamepad1.right_bumper || gamepad1.right_trigger > 0.5) picked = Side.RED;
//        if (gamepad1.left_bumper  || gamepad1.left_trigger  > 0.5) picked = Side.BLUE;
//        if (picked != side) {                // new alliance = new tag range, forget any old tag
//            side = picked;
//            tagSeen = false;
//            seenTagId = -1;
//        }
//
//        telemetry.addData("Alliance", allianceLabel());
//        telemetry.addData("APRILTAG", tagSeen ? "SEEN (id " + seenTagId + ")" : "NOT SEEN");
//        telemetry.addData("Visible tag IDs", visibleTagIds());
//        telemetry.addData("Turret deg / target", "%.1f / %.1f", turret.getPositionDeg(), turret.getTargetDeg());
//        telemetry.addLine(side == null ? "NOT READY" : "READY: press START");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        autoTimer.reset();
//        follower.setPose(start);
//        intake.start();                                  // intake off, flower/front/back all up
//        depo.start();                                    // hatch closed, depo off
//        turret.setPositionDeg(TURRET_START_DEG);         // set the target again at start
//        turret.start();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        turret.run_using_pid();
//        depo.run_using_pid();
//        updatePathState();
////        checkFailsafe();
//
//        telemetry.addData("Alliance", allianceLabel());
//        telemetry.addData("APRILTAG", tagSeen ? "SEEN (id " + seenTagId + ")" : "NOT SEEN");
//        telemetry.addData("Visible tag IDs", visibleTagIds());
//        telemetry.addData("Path / Action", pathState + " / " + actionState);
//        telemetry.addData("Auto timer", "%.1f / %.0f", autoTimer.seconds(), PARK_TIME_SECONDS);
//        telemetry.addData("Depo velocity / target", "%.0f / %.0f", depo.getVelocity(), SHOT_VELOCITY);
//        telemetry.addData("Turret deg / target", "%.1f / %.1f", turret.getPositionDeg(), turret.getTargetDeg());
//        telemetry.addData("Pose", follower.pose());
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        if (portal != null) { portal.close(); portal = null; }   // close the camera first so it always gets released
//        shooterOff();
//        turret.turretOff();
//    }
//
//    // ---------------- PATH STATE MACHINE ----------------
//    // Intake servos by phase:
//    //   start -> leaveStart (incl. preload shot): flower UP, front UP, back UP
//    //   leaveStart -> flower -> until shootPose:  flower DOWN, front UP, back DOWN
//    //   driving to / at the shootPose:            flower DOWN, front DOWN, back DOWN
//    //   after the shot (IDLE):                    everything retracted for parking
//    private void updatePathState() {
//        switch (pathState) {
//            case 1:                                   // nothing moves till the AprilTag is seen
//                for (AprilTagDetection d : tagProcessor.getDetections()) {
//                    if (d.id ==30 || d.id==31 || d.id==32 || d.id==33 || d.id==38 || d.id==39 || d.id==40 || d.id==41) {
//                        pathState = 2;
//                        depo.set_target_velocity(1500);
//                        didytimer.reset();
//                        portal.close();                                  // tag locked: stop streaming + detection, free the camera
////                        portal = null;
//                    }
//                }
//                break;
//
//            case 2:
//                if(didytimer.seconds()>1.5){
//                    intake.intake_on();
//                    depo.open_hatch();
//                    didytimer.reset();
//                    pathState=3;
//                }
//                break;
//
//            case 3:
//                if(didytimer.seconds()>2){
//                    depo.close_hatch();
//                    depo.turn_off_deposit();
//                    follower.follow(path1());
//                    pathState=15;
//                }
//
//            case 4:
//                if (!follower.isBusy()) {
//                    intake.flower_down();
//                    follower.follow(path2());
//                    pathState=5;
//                }
//                break;
//
//            case 5:
//                if (!follower.isBusy()) {
//                    didytimer.reset();
//                    pathState=6;
//                }
//                break;
//
//            case 6:
//                if (didytimer.seconds()>1.5) {
//                    follower.follow(path3());
//                    intake.front_down();
//                    depo.set_target_velocity(1500);
//                    pathState=7;
//                }
//                break;
//
//            case 7:// cancel everything and park
//                if (!follower.isBusy()) {
//                    didytimer.reset();
//                    pathState=8;
//                }
//                break;
//
//            case 8:
//                if (didytimer.seconds()>1.5){
//                    depo.open_hatch();
//                    didytimer.reset();
//                    pathState=9;
//                }
//                break;
//            case 9:
//                if(didytimer.seconds()>2){
//                    depo.close_hatch();
//                    intake.intake_off();
//                    intake.front_up();
//                    intake.flower_up();
//                    follower.follow(path4());
//                    pathState=10;
//                }
//                break;
//        }
//    }
//
//    // ---------------- ACTIONS ----------------
//    /** While shooting: wait until the robot has stopped, then run the shot. Back to IDLE when it finishes. */
//
//
//    /** Intake pose for collecting: flower DOWN, front UP, back DOWN, rollers on. */
//    private void collectBalls() {
//        intake.flower_down();
//        intake.front_up();
//        intake.back_down();
//        intake.intake_on();
//    }
//
//    private boolean finishShot() {
//        if (depo.reachedvelocity() && !timers.isRunning("shoot")) {
//            timers.start("shoot");
//        }
//        if (timers.checkSeconds("shoot", SHOT_OPEN_SECONDS)) {
//            intake.intake_on();
//            depo.open_hatch();
//        }
//        if (timers.checkSecondsLast("shoot", SHOOT_HOLD_SECONDS)) {
//            shooterOff();                                        // hatch closed, depo off, intake off
//            return true;
//        }
//        return false;
//    }
//
//    /** Turns the flywheel on at the shot velocity (Depo's default + SHOT_VELOCITY_BOOST). */
//    private void spinUpShooter() {
//        depo.turn_on_deposit();                                  // resets the PID and turns the depo on
//        depo.set_target_velocity(SHOT_VELOCITY);                 // then raise the target
//    }
//
//    private void shooterOff() {
//        depo.close_hatch();
//        depo.turn_off_deposit();
//        intake.intake_off();
//    }
//
//    // ---------------- 27s FAILSAFE: leave start, then park ----------------
//    private void checkFailsafe() {
//        if (autoTimer.seconds() > PARK_TIME_SECONDS && !autoaura) {
//            // everything off
//            autoaura=true;
//            depo.turn_off_deposit();
//            intake.intake_off();
//            intake.front_up();
//            intake.flower_up();
//            intake.back_up();
//            follower.follow(drive(follower.pose(), point4));     // leg 1: wherever we are -> leave start
//            pathState = 0;                    // leg 2 (-> park) runs in FAILSAFE_LEAVE
//        }
//    }
//
//    // ---------------- APRILTAG ----------------
//    private void checkForTag() {
//        if (tagSeen || side == null || portal == null) return;
//        for (AprilTagDetection d : tagProcessor.getDetections()) {
//            if (d.id >= side.minTag && d.id <= side.maxTag) {
//                tagSeen = true;
//                seenTagId = d.id;
//                portal.close();                                  // tag locked: stop streaming + detection, free the camera
//                portal = null;
//                return;
//            }
//        }
//    }
//
//    private String visibleTagIds() {
//        if (portal == null) return "camera off";                 // portal closed after the tag was locked
//        StringBuilder sb = new StringBuilder();
//        for (AprilTagDetection d : tagProcessor.getDetections()) sb.append(d.id).append(' ');
//        return sb.length() == 0 ? "none" : sb.toString().trim();
//    }
//
//    private String allianceLabel() {
//        return side == null ? "NONE - press RIGHT (red) or LEFT (blue)"
//                : side + " (tags " + side.minTag + "-" + side.maxTag + ")";
//    }
//}