package org.firstinspires.ftc.teamcode.OFSB2.Auto.Season.LM0;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.OFSB2.Subsystems.turret;

import org.firstinspires.ftc.teamcode.OFSB2.Subsystems.intake;
import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;

@Autonomous(name = "AutoLM0", group = "Season")
public class AutoLM0 extends OpMode {

    private DcMotorEx depo, depo1, depo2;
    private Servo lift_left, turret, //turret2,
    launch_amgle;
    private CRServo turret_servo;
    private Follower follower;
    Pose start, sample1, shoot, sample2, done;
    private PathChain startToSample1, sample1ToShoot, shootToSample2, sample2ToDone;

    private Timer pathTimer, opModeTimer;

    boolean isBlue, isRed;

    private enum AutoState {
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
    public void subsystems() {
        intake intake = new intake(hardwareMap);
        turret turret = new turret(hardwareMap);

    }

    @Override
    public void init() {
        subsystems();
        hardwareMap();
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        autoState = AutoState.STARTTOSAMPLE1;

        telemetry.addLine("Ready. Press Y for Blue or A for Red.");
    }

    @Override
    public void init_loop() {
        gamepadSide();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        updateAutoState();
        updateTelemetry();
    }

    public void hardwareMap() {
    }

    // --- THE SINGLE STATE MACHINE ---
    private void updateAutoState() {
        // If the robot is currently driving a path, wait here and do nothing
        if (follower.isBusy()) {
            return;
        }

        // When the current path finishes, trigger the next path and update the state to match
        switch (autoState) {
            case STARTTOSAMPLE1:
                // Finished startToSample1 -> drive sample1ToShoot
                follower.followPath(startToSample1, true);
                setPathState(AutoState.SAMPLE1TOSHOOT);
                break;

            case SAMPLE1TOSHOOT:
                // Finished sample1ToShoot -> drive shootToSample2
                follower.followPath(sample1ToShoot, true);
                setPathState(AutoState.SHOOTTOSAMPLE2);
                break;

            case SHOOTTOSAMPLE2:
                // Finished shootToSample2 -> drive sample2ToDone
                follower.followPath(shootToSample2, true);
                setPathState(AutoState.SAMPLE2TODONE);
                break;

            case SAMPLE2TODONE:
                // Finished sample2ToDone
                follower.followPath(sample2ToDone, true);
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
            case TURRET:

        }
    }

    // Helper method to set the state and reset the timer cleanly
    private void setPathState(AutoState newState) {
        autoState = newState;
        pathTimer.resetTimer();
    }

    private void poses() {
        if (isBlue) {
            start = new Pose(84.35853658536585, 134.31463414634146, Math.toRadians(0));
            sample1 = new Pose(126.23658536585364, 134.70487804878047, Math.toRadians(0));
            shoot = new Pose(90.10487804878049, 36.74390243902439, Math.toRadians(-90));
            sample2 = new Pose(100.72334963325183, 16.883481424056274, Math.toRadians(-90));
            done = new Pose(138.7166915141034, 33.84915618104835, Math.toRadians(180));
        }

        else if (isRed) {
            start = new Pose(59.64146341463415, 9.68536585365854, Math.toRadians(180));
            sample1 = new Pose(17.76341463414636, 9.29512195121953, Math.toRadians(180));
            shoot = new Pose(53.89512195121951, 107.25609756097561, Math.toRadians(90));
            sample2 = new Pose(43.27665036674817, 127.11651857594373, Math.toRadians(90));
            done = new Pose(5.283308485896602, 110.15084381895165, Math.toRadians(0));

        }
    }

    private void buildPaths() {
        if (isBlue) {
            startToSample1 = follower.pathBuilder()
                    .addPath(new BezierLine(start, sample1))
                    .setLinearHeadingInterpolation(start.getHeading(), sample1.getHeading())
                    .build();

            sample1ToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(sample1, new Pose(125.05243902439025, 65.78780487804879), new Pose(112.61829268292684, 29.90365853658537), shoot))
                    .setLinearHeadingInterpolation(sample1.getHeading(), shoot.getHeading())
                    .build();

            shootToSample2 = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot, new Pose(94.88536585365854, 25.947560975609758), sample2))
                    .setLinearHeadingInterpolation(shoot.getHeading(), sample2.getHeading())
                    .build();

            sample2ToDone = follower.pathBuilder()
                    .addPath(new BezierCurve(sample2, new Pose(98.33658536585365, 38.026829268292694), done))
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
    }

    public void gamepadSide() {
        if (gamepad1.y) {
            isBlue = true;
            isRed = false;
            poses();
            buildPaths();
            follower.setPose(start);
            telemetry.addData("Selected", "BLUE");
        }
        else if(gamepad1.a) {
            isRed = true;
            isBlue = false;
            poses();
            buildPaths();
            follower.setPose(start);
            telemetry.addData("Selected", "RED");
        }
    }

    private void updateTelemetry() {
        telemetry.addData("auto state", autoState);
        Pose pose = follower.getPose();
        if (pose != null) {
            telemetry.addData("x", pose.getX());
            telemetry.addData("y", pose.getY());
            telemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
        }
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());
    }
}