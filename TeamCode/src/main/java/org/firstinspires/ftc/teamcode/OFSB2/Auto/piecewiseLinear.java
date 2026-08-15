package org.firstinspires.ftc.teamcode.OFSB2.Auto;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "piecewiseLinear", group = "Autonomous")
public class piecewiseLinear extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // --- EMBEDDED CONSTANTS (Optimized for Movement & Precision) ---
    public FollowerConstants followerConstants = new FollowerConstants()
            .mass(7.2)
            .forwardZeroPowerAcceleration(-39.370416251310814)
            .lateralZeroPowerAcceleration(-56.98433031510037)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.06, 0.07))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.01, 0.019))
            .centripetalScaling(0.0005)
            .BEZIER_CURVE_SEARCH_LIMIT(100);

    public MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.6)
            .rightFrontMotorName("fr")
            .rightRearMotorName("rr")
            .leftRearMotorName("rl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(95.27855676365651)
            .yVelocity(73.45713601900836);

    public PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.5)
            .strafePodX(7)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("imu")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public PathConstraints pathConstraints = new PathConstraints(0.99, 0.1, 0.1, 0.007, 100, 1.2, 100, 1);

    public enum PathState {
        START,
        END,
        DONE
    }

    PathState pathState;

    private final Pose startingCoordinate = new Pose(72, 7, Math.toRadians(90));
    private final Pose path1complete = new Pose(72, 100, Math.toRadians(180));

    private PathChain start_finish; 

    public void buildPaths() {
        start_finish = follower.pathBuilder()
                .addPath(new BezierLine(startingCoordinate, path1complete))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0, .5, HeadingInterpolator.tangent),
                        new HeadingInterpolator.PiecewiseNode(
                                .5, 1, HeadingInterpolator.linear(Math.toRadians(90), Math.toRadians(180)))))
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START:
                follower.followPath(start_finish, true);
                setPathState(PathState.DONE);
                break;
/*
            case END:
                // Safeguard: Ensure robot has moved past the start (T > 0.1) before allowed to finish
                if (follower.getCurrentTValue() > 0.1 && follower.atParametricEnd()) {
                    setPathState(PathState.DONE);
                }
                break;
                */
            case DONE:
                if (!follower.isBusy()) {
                    telemetry.addLine("Fully Linear Piecewise Loop Finished!");
                }
                break;
            default:
                telemetry.addLine("State machine error");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();

        buildPaths();
        follower.setPose(startingCoordinate);
        
        pathState = PathState.START;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
