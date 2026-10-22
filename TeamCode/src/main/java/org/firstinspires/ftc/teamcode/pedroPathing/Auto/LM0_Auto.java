package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

//import com.pedropathing.ivy.Scheduler;
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.groups.Groups.sequential;
//import static com.pedropathing.ivy.pedro.PedroCommands.*;


import static org.firstinspires.ftc.teamcode.pedroPathing.subsystems.turret.turretAngle;

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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.CustomFollower;
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
    private turret turret;
    private Servo hatch;
    private CRServo turret_servo;
    private CustomFollower follower;
    Pose start, sample1, point2, shoot, sample2, done;
    private PathChain startToSample1, sample1ToPoint2, sample1ToShoot, shootToDone;

    private Timer pathTimer, opModeTimer;
    // pedroPathing's Timer is already imported above, so the team's own Timer (same one soloTeleop uses) needs its full name
    private org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer timers;

    boolean isBlue, isRed, isSmallBlue, isSmallRed, isBlueSample, isRedSample, isBlue2;
    //intake intake;
    // turret turret;

    // soloTeleop teleop = new soloTeleop();

    private enum AutoState {
        STARTTOSAMPLE1,
        SAMPLE1TOPOINT2,
        SAMPLE1TOSHOOT,
        // SHOOTTOSAMPLE2,
        SHOOTTODONE,
        DONE
    }

    private enum ShootState {
        INTAKE,
        SHOOT,
        SHOOTOFFSET,
        TURRET,
        KICK;
    }

    private AutoState autoState;
    private ShootState shootState;
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
        telemetry.addLine("Ready. Press Y for Blue, A for Red.");
        telemetry.addLine("Press X for Small Blue, B for Small Red.");
        telemetry.addLine("Press D-pad UP for Blue Sample, D-pad DOWN for Red Sample.");
        telemetry.addLine("Press D-pad LEFT for Blue 2.");
    }

    @Override
    public void init_loop() {
        gamepadSide();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        intake.start();
        depo.start();
        turret.ResetTurretEncoders();
        timers.createNew("depo");
        setShootState(ShootState.SHOOT);
        depo.turn_on_deposit();
    }

    @Override
    public void loop() {
        follower.update();
        turret.run_using_pid();
        depo.run_using_pid();
        updateAutoState();
        updateTelemetry();
        updateShootState();
    }

    // --- THE SINGLE STATE MACHINE ---
    private void updateAutoState() {
        if (follower.isBusy()) {
            return;
        }

        switch (autoState) {
            case STARTTOSAMPLE1:
                if (pathTimer.getElapsedTimeSeconds() > 1.25) {
                    setShootState(ShootState.INTAKE);
                    follower.acceleration(0.5, 1.0, 1.0, 0.5);
                    follower.followPath(startToSample1, true);
                    if (isBlue2) {
                        setPathState(AutoState.SAMPLE1TOPOINT2);
                    }
                    else {
                        setPathState(AutoState.SAMPLE1TOSHOOT);
                    }
                }
                break;

            case SAMPLE1TOPOINT2:
                follower.followPath(sample1ToPoint2, true);
                setPathState(AutoState.SAMPLE1TOSHOOT);
                break;

            case SAMPLE1TOSHOOT:
                follower.followPath(sample1ToShoot, true);
                setPathState(AutoState.SHOOTTODONE);
                break;

            /* case SHOOTTOSAMPLE2:
                follower.followPath(shootToSample2, true);
                setPathState(AutoState.SAMPLE2TODONE);
                break;

             */

            case SHOOTTODONE:
                if (isBlue || isRed || isBlueSample || isRedSample || isBlue2) {
                    follower.followPath(shootToDone, true);
                }
                else {
                    // This executes for Small paths AND the Sample paths
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

    private void updateShootState() {
        switch (shootState) {
            case INTAKE:
                //needDepo = false;
                intake.turn_on_intake();
                break;
            case SHOOT:
                //needDepo = true;
                turret.setPositionDeg(90);
                intake.turn_on_intake();
                if(turret.getPositionDeg() >= 90) {
                    runAutoDepo();
                }
                break;
            case SHOOTOFFSET:
                //needDepo = true;
                double offsetDeg = turretAngle(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
                turret.setPositionDeg(offsetDeg);
                intake.turn_on_intake();
                if(turret.getPositionDeg() >= offsetDeg) {
                    runAutoDepo();
                }
                break;

        }
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
            start = new Pose(84.35853658536585, 134.31463414634146, Math.toRadians(0));
            sample1 = new Pose(126.23658536585364, 134.70487804878047, Math.toRadians(0));
            shoot = new Pose(90.71481304788598, 14.812749716739198, Math.toRadians(-90));
            done = new Pose(138.7166915141034, 33.84915618104835, Math.toRadians(180));
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
        } else if (isBlue2) {
            start = new Pose(108.3585, 134.3146, Math.toRadians(0));
            sample1 = new Pose(126.23658536585364, 134.70487804878047, Math.toRadians(0));
            point2 = new Pose(108.3585, 134.3146, Math.toRadians(0));
            shoot = new Pose(90.71481304788598, 14.812749716739198, Math.toRadians(-90));
            done = new Pose(138.7166915141034, 33.84915618104835, Math.toRadians(180));
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
                    .setHeadingInterpolation(HeadingInterpolator.piecewise(new HeadingInterpolator.PiecewiseNode(0, 0.15, HeadingInterpolator.linear(sample1.getHeading(), shoot.getHeading())),
                            new HeadingInterpolator.PiecewiseNode(0.15, 1, HeadingInterpolator.constant(Math.toRadians(-90)))))
                    .build();

            shootToDone = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot, new Pose(98.33658536585365, 38.026829268292694), done))
                    .setLinearHeadingInterpolation(shoot.getHeading(), done.getHeading())
                    .build();
        }

        else if(isRed) {
            startToSample1 = follower.pathBuilder()
                    .addPath(new BezierLine(start, sample1))
                    .setLinearHeadingInterpolation(start.getHeading(), sample1.getHeading())
                    .build();

            sample1ToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(sample1, new Pose(18.94756097560975, 78.21219512195121), new Pose(31.38170731707316, 114.09634146341463), shoot))
                    .setHeadingInterpolation(HeadingInterpolator.piecewise(new HeadingInterpolator.PiecewiseNode(0, 0.15, HeadingInterpolator.linear(sample1.getHeading(), shoot.getHeading())),
                            new HeadingInterpolator.PiecewiseNode(0.15, 1, HeadingInterpolator.constant(Math.toRadians(90)))))
                    .build();

            shootToDone = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot, new Pose(45.66341463414635, 105.97317073170731), done))
                    .setLinearHeadingInterpolation(shoot.getHeading(), done.getHeading())
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
            sample1ToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(start, new Pose(88.3183, 28.3902), sample1))
                    .setLinearHeadingInterpolation(start.getHeading(), sample1.getHeading())
                    .build();

            shootToDone = follower.pathBuilder()
                    .addPath(new BezierLine(sample1, done))
                    .setLinearHeadingInterpolation(sample1.getHeading(), done.getHeading())
                    .build();
        }
        else if(isRedSample) {
            sample1ToShoot = follower.pathBuilder()
                    // Exactly reflected control point (144 - X, 144 - Y)
                    .addPath(new BezierCurve(start, new Pose(55.6817, 115.6098), sample1))
                    .setLinearHeadingInterpolation(start.getHeading(), sample1.getHeading())
                    .build();

            shootToDone = follower.pathBuilder()
                    .addPath(new BezierLine(sample1, done))
                    .setLinearHeadingInterpolation(sample1.getHeading(), done.getHeading())
                    .build();
        }
        else if(isBlue2) {
            startToSample1 = follower.pathBuilder()
                    .addPath(new BezierLine(start, sample1))
                    .setLinearHeadingInterpolation(start.getHeading(), sample1.getHeading())
                    .build();

            sample1ToPoint2 = follower.pathBuilder()
                    .addPath(new BezierLine(sample1, point2))
                    .setLinearHeadingInterpolation(sample1.getHeading(), point2.getHeading())
                    .build();

            sample1ToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(point2, new Pose(118.3537, 26.6378), shoot))
                    .setHeadingInterpolation(HeadingInterpolator.piecewise(new HeadingInterpolator.PiecewiseNode(0, 0.15, HeadingInterpolator.linear(point2.getHeading(), shoot.getHeading())),
                            new HeadingInterpolator.PiecewiseNode(0.15, 1, HeadingInterpolator.constant(Math.toRadians(-90)))))
                    .build();

            shootToDone = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot, new Pose(98.33658536585365, 38.026829268292694), done))
                    .setLinearHeadingInterpolation(shoot.getHeading(), done.getHeading())
                    .build();
        }
    }

    public void gamepadSide() {
        if (gamepad1.y) {
            isBlue = true; isRed = false; isSmallBlue = false; isSmallRed = false; isBlueSample = false; isRedSample = false; isBlue2 = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.STARTTOSAMPLE1;
            telemetry.addData("Selected", "BLUE");
        }
        else if(gamepad1.a) {
            isRed = true; isBlue = false; isSmallBlue = false; isSmallRed = false; isBlueSample = false; isRedSample = false; isBlue2 = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.STARTTOSAMPLE1;
            telemetry.addData("Selected", "RED");
        }
        else if(gamepad1.x) {
            isSmallBlue = true; isBlue = false; isRed = false; isSmallRed = false; isBlueSample = false; isRedSample = false; isBlue2 = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.SHOOTTODONE;
            telemetry.addData("Selected", "SMALL BLUE");
        }
        else if(gamepad1.b) {
            isSmallRed = true; isSmallBlue = false; isBlue = false; isRed = false; isBlueSample = false; isRedSample = false; isBlue2 = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.SHOOTTODONE;
            telemetry.addData("Selected", "SMALL RED");
        }
        else if(gamepad1.dpad_up) {
            isBlueSample = true; isBlue = false; isRed = false; isSmallBlue = false; isSmallRed = false; isRedSample = false; isBlue2 = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.SAMPLE1TOSHOOT;
            telemetry.addData("Selected", "BLUE SAMPLE");
        }
        else if(gamepad1.dpad_down) {
            isRedSample = true; isBlue = false; isRed = false; isSmallBlue = false; isSmallRed = false; isBlueSample = false; isBlue2 = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.SAMPLE1TOSHOOT;
            telemetry.addData("Selected", "RED SAMPLE");
        }
        else if(gamepad1.dpad_left) {
            isBlue2 = true; isBlue = false; isRed = false; isSmallBlue = false; isSmallRed = false; isBlueSample = false; isRedSample = false;
            poses();
            buildPaths();
            follower.setPose(start);
            autoState = AutoState.STARTTOSAMPLE1;
            telemetry.addData("Selected", "BLUE 2");
        }
    }

    public void runAutoDepo() {
        depo.run_using_pid();

        if (depo.reachedvelocity() && !timers.isRunning("depo")){
            timers.start("depo");
            depo.open_hatch();
            intake.turn_on_intake();
        }
        if (timers.checkSecondsLast("depo", 3)){
            depo.turn_off_deposit();
            depo.close_hatch();

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