package org.firstinspires.ftc.teamcode.OFSB2.Auto.Season.LM0;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;

@Autonomous(name = "AutoLM0", group = "Season")
public class AutoLM0 extends OpMode {
    private Follower follower;
    Pose start, shoot, sample1, shoot2, sample2, shoot3;
    private PathChain startToShoot, shootToSample1, sample1Toshoot2, shoot2ToSample2, sample2ToShoot3;

    private Timer pathTimer, opModeTimer;

    private enum AutoState {
        DRIVETOSHOOT,
        SHOOTTOSAMPLE1,
        SAMPLE1TOSHOOT2,
        SHOOT2TOSAMPLE2,
        SAMPLE2TOSHOOT3,
        DONE
    }

    private AutoState autoState;

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        poses();
        buildPaths();
        follower.setPose(start);
        setAutoState(AutoState.DRIVETOSHOOT);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setAutoState(autoState);
    }

    @Override
    public void loop() {
        follower.update();
        updateAutoState();
        updateTelemetry();
    }

    private void updateAutoState() {
        switch (autoState) {
            case DRIVETOSHOOT:
                handleDriveToShoot();
                break;
            case SHOOTTOSAMPLE1:
                handleShootToSample1();
                break;
            case SAMPLE1TOSHOOT2:
                handleSample1ToShoot2();
                break;
            case SHOOT2TOSAMPLE2:
                handleShoot2ToSample2();
                break;
            case SAMPLE2TOSHOOT3:
                handleSample2ToShoot3();
                break;
            case DONE:
                handleDone();
                break;
            default:
                telemetry.addLine("State machine not working");
                break;
        }
    }

    private void handleDriveToShoot() {
        follower.followPath(startToShoot, true);
        if (!follower.isBusy()) {
            setAutoState(AutoState.SHOOTTOSAMPLE1);
        }
    }

    private void handleShootToSample1() {
            follower.followPath(shootToSample1, true);
            if (!follower.isBusy()) {
                setAutoState(AutoState.SAMPLE1TOSHOOT2);
        }
    }

    private void handleSample1ToShoot2() {
            follower.followPath(sample1Toshoot2, true);
            if (!follower.isBusy()) {
                setAutoState(AutoState.SHOOT2TOSAMPLE2);
        }
    }

    private void handleShoot2ToSample2() {
            follower.followPath(shoot2ToSample2, true);
            if (!follower.isBusy()) {
                setAutoState(AutoState.SAMPLE2TOSHOOT3);
        }
    }

    private void handleSample2ToShoot3() {
            follower.followPath(sample2ToShoot3, true);
            if (!follower.isBusy()) {
                setAutoState(AutoState.DONE);
        }
    }

    private void handleDone() {
        if (!follower.isBusy()) {
            telemetry.addLine("Finished everything!");
        }
    }

    private void setAutoState(AutoState newState) {
        autoState = newState;
        pathTimer.resetTimer();
    }

    private void poses() {
        start = new Pose(83.6920731707317, 134.18341568737495, Math.toRadians(270));
        shoot = new Pose(84.29999999999998, 115.72317073170733, Math.toRadians(270));
        sample1 = new Pose(133.01585365853657, 133.9451219512195, Math.toRadians(90));
        shoot2 = new Pose(80.15975609756097, 23.434146341463418);
        sample2 = new Pose(92.00853658536587, 7.939024390243908, Math.toRadians(-50));
        shoot3 = shoot2;
    }

    private void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(start, shoot))
                .setLinearHeadingInterpolation(start.getHeading(), shoot.getHeading())
                .build();

        BezierCurve shootToSampleCurve = new BezierCurve(shoot, new Pose(83.63780487804877, 95.71707317073171, 0), new Pose(131.0951219512195, 105.77682926829266, 0), sample1);
        double tangentAt086 = shootToSampleCurve.getDerivative(0.86).getTheta();
        shootToSample1 = follower.pathBuilder()
                .addPath(shootToSampleCurve)
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0.0, 0.86, HeadingInterpolator.tangent),
                        new HeadingInterpolator.PiecewiseNode(0.86, 1.0, HeadingInterpolator.linear(tangentAt086, Math.toRadians(-50)))))
                .build();

        BezierCurve sample1ToShoot2Curve = new BezierCurve(sample1, new Pose(121.20243902439023, 41.75365853658536, 0), new Pose(85.65365853658535, 23.400000000000002, 0), shoot2);
        sample1Toshoot2 = follower.pathBuilder()
                .addPath(sample1ToShoot2Curve)
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();

        BezierLine shoot2ToSample2Line = new BezierLine(shoot2, sample2);
        double shoot2StartHeading = MathFunctions.normalizeAngle(sample1ToShoot2Curve.getDerivative(1.0).getTheta() + Math.PI);
        double shoot2LineEndHeading = shoot2ToSample2Line.getDerivative(1.0).getTheta();
        shoot2ToSample2 = follower.pathBuilder()
                .addPath(shoot2ToSample2Line)
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0.0, 0.5, HeadingInterpolator.linear(shoot2StartHeading, 0)),
                        new HeadingInterpolator.PiecewiseNode(0.5, 1.0, HeadingInterpolator.linear(0, shoot2LineEndHeading))))
                .build();

        sample2ToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(sample2, shoot3))
                .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                .build();
    }

    private void updateTelemetry() {
        telemetry.addData("auto state", autoState);
        Pose pose = follower.getPose();
        if (pose != null) {
            telemetry.addData("x", pose.getX());
            telemetry.addData("y", pose.getY());
            telemetry.addData("heading", pose.getHeading());
        }
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());
    }
}
