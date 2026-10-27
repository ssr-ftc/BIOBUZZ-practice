package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Bottom_Auto", group = "Auto")
public class Bottom_Auto_Aditi extends OpMode {

    // =====================================================================
    //  CONFIG
    // =====================================================================
    private static final String CAMERA_NAME = "camera";      // must match robot config
    private static final double PARK_TIME_SECONDS = 27.0;    // failsafe: go straight to park

    private static final int RED_MIN = 30,  RED_MAX = 33;    // red side tags
    private static final int BLUE_MIN = 38, BLUE_MAX = 41;   // blue side tags

    private static final double INTAKE_TIME_SECONDS = 1.0;   // TODO tune
    private static final double SHOOT_TIME_SECONDS  = 1.5;   // TODO tune

    // =====================================================================
    //  POSES  (Pedro Visualizer, heading in degrees)
    // =====================================================================
    private static Pose pose(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }
    private final Pose start       = pose(64.7738, 133.9711, 90);
    private final Pose gotoFlower  = pose(47.0635, 122.0681, 90);
    private final Pose flower      = pose(47.1237, 130.5576, 90);
    private final Pose shootFlower = pose(47.2164, 122.2292, 290);
    private final Pose park        = pose(7.8753, 114.1343, 0);

    // =====================================================================
    //  PATH STATE
    // =====================================================================
    private enum PathState {
        WAIT_FOR_TAG,    // sit still until the correct AprilTag is seen
        GO_TO_FLOWER,    // start -> gotoflower
        FLOWER,          // gotoflower -> flower   (intake runs)
        SHOOT_FLOWER,    // flower -> shootflower
        SHOOTING,        // sit at shootflower while the shot happens
        PARK,            // shootflower -> park (or direct park from failsafe)
        DONE
    }
    private PathState pathState = PathState.WAIT_FOR_TAG;
    private final ElapsedTime pathTimer = new ElapsedTime();

    // =====================================================================
    //  ACTION STATE
    // =====================================================================
    private enum ActionState {
        IDLE,       // nothing running
        INTAKE,     // intake running for INTAKE_TIME_SECONDS, then back to IDLE
        SHOOT,      // shooter running for SHOOT_TIME_SECONDS, then back to IDLE
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

    private Follower follower;
    private PathChain gotoFlowerPath, flowerPath, shootFlowerPath, parkPath;

    private AprilTagProcessor tagProcessor;
    private VisionPortal portal;
    private final ElapsedTime autoTimer = new ElapsedTime();

    // TODO: declare your mechanisms here, e.g.
    // private DcMotorEx intake, shooter;

    // =====================================================================
    //  OPMODE LIFECYCLE
    // =====================================================================
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        buildPaths();

        tagProcessor = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .addProcessor(tagProcessor)
                .build();

        // TODO: hardwareMap.get(...) for your mechanisms here

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

        sendTelemetry();
    }

    @Override
    public void stop() {
        if (portal != null) portal.close();
    }

    // =====================================================================
    //  PATH STATE MACHINE
    // =====================================================================
    private void updatePathState() {
        switch (pathState) {

            case WAIT_FOR_TAG:
                if (tagSeen) {
                    follower.followPath(gotoFlowerPath, true);
                    setPathState(PathState.GO_TO_FLOWER);
                }
                break;

            case GO_TO_FLOWER:
                if (!follower.isBusy()) {
                    follower.followPath(flowerPath, true);
                    setActionState(ActionState.INTAKE);          // intake while going to the flower
                    setPathState(PathState.FLOWER);
                }
                break;

            case FLOWER:
                // wait for BOTH the path and the intake to finish
                if (!follower.isBusy() && actionState == ActionState.IDLE) {
                    follower.followPath(shootFlowerPath, true);
                    setPathState(PathState.SHOOT_FLOWER);
                }
                break;

            case SHOOT_FLOWER:
                if (!follower.isBusy()) {
                    setActionState(ActionState.SHOOT);
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
                // TODO: make sure everything is off
                // intake.setPower(0); shooter.setPower(0);
                break;

            case INTAKE:
                // TODO: intake.setPower(1.0);
                if (actionTimer.seconds() >= INTAKE_TIME_SECONDS) {
                    // TODO: intake.setPower(0);
                    setActionState(ActionState.IDLE);
                }
                break;

            case SHOOT:
                // TODO: shooter.setPower(1.0); feed when up to speed
                if (actionTimer.seconds() >= SHOOT_TIME_SECONDS) {
                    // TODO: shooter.setPower(0);
                    setActionState(ActionState.IDLE);
                }
                break;

            case STOPPED:
                // TODO: intake.setPower(0); shooter.setPower(0);
                break;
        }
    }

    private void setActionState(ActionState newState) {
        actionState = newState;
        actionTimer.reset();
    }

    // =====================================================================
    //  27s FAILSAFE
    // =====================================================================
    private void checkFailsafe() {
        boolean alreadyParking = pathState == PathState.PARK || pathState == PathState.DONE;
        if (autoTimer.seconds() >= PARK_TIME_SECONDS && !forcedPark && !alreadyParking) {
            forcedPark = true;

            follower.breakFollowing();                 // cancel the current path
            setActionState(ActionState.STOPPED);       // stop intake/shooter

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
        telemetry.addData("Forced Park", forcedPark);
        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
    }
}