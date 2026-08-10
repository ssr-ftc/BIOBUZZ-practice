package org.firstinspires.ftc.teamcode.OFSWB;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            // Weigh the robot (with battery) on a scale, convert to your preferred unit.
            .mass(12.29235)

            // ---- Zero Power Accelerations ----
            // SOURCE: Tuning OpMode -> Automatic -> Forward Zero Power Acceleration Tuner
            // Robot speeds up then cuts power; telemetry reports "Deceleration" -> copy here.
            .forwardZeroPowerAcceleration(-40.20392786649596)

            // SOURCE: Tuning OpMode -> Automatic -> Lateral Zero Power Acceleration Tuner
            // Same as above but strafing left/right.
            .lateralZeroPowerAcceleration(-68.75878748434566)

            // ---- Translational PID ----
            // SOURCE: FTC Dashboard / Panels, live-tuned.
            // Run StraightBackAndForth (or Line Tuner) with useHeading OFF, translational ON.
            // Manually push the robot off-line and adjust P/I/D/F in the dashboard until it
            // snaps back quickly without oscillating. Copy the final values here.
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.8, // P
                    0, // I
                    0.000001, // D
                    1  // F
            ))
            // SOURCE: Also set in the dashboard during translational tuning -- this is the
            // error distance at which it switches from primary to secondary PID.
            .translationalPIDFSwitch(0)
            // SOURCE: Same dashboard session, tuned AFTER the primary PID above, using small
            // nudges instead of large ones (cleans up small residual error).
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0, // P
                    0, // I
                    0, // D
                    0  // F
            ))

            // ---- Heading PID ----
            // SOURCE: FTC Dashboard / Panels, live-tuned.
            // Run StraightBackAndForth with useHeading ON, other corrections OFF.
            // Manually rotate the robot and adjust P/I/D/F until it corrects heading smoothly
            // without drifting laterally or oscillating. Copy the final values here.
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0, // P
                    0, // I
                    0., // D
                    0  // F
            ))
            // SOURCE: Same dashboard session, tuned after the primary heading PID, using
            // small rotational nudges.
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    0, // P
                    0, // I
                    0, // D
                    0  // F
            ))

            // ---- Predictive Braking (replaces Drive PIDF) ----
            // SOURCE: Tuning.java -> Automatic -> PredictiveBrakingTuner gives you
            // kLinear and kQuadratic. kP is not from the tuner -- start around 0.1 and
            // adjust by feel on the Line Test (usual range 0.05-0.3).
            // Args are (kP, kLinear, kQuadratic).
            //   kQuadratic: braking distance proportional to velocity^2 (braking power,
            //               sliding friction)
            //   kLinear:    braking distance roughly proportional to velocity (back-EMF,
            //               torque delay, viscous friction)
            // PLACEHOLDER VALUES BELOW -- run PredictiveBrakingTuner and replace kLinear/
            // kQuadratic with your actual results before trusting this in auto.
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(
                    0.1,    // kP        (placeholder -- tune by feel on Line Test)
                    0.05535984547289155,   // kLinear   (placeholder -- replace with PredictiveBrakingTuner output)
                    0.0017232309268316959  // kQuadratic (placeholder -- replace with PredictiveBrakingTuner output)
            ))

            // ---- Centripetal Scaling ----
            // Kept at 0 intentionally: Pedro Pathing recommends turning centripetal
            // scaling OFF when using Predictive Braking, since it already accounts for
            // cornering forces on its own.
            .centripetalScaling(0);

    // SOURCE: These 4 numbers are your path-following tolerances/limits. Confirm which
    // constructor overload your installed Pedro Pathing version uses -- current docs show:
    // (tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint,
    //  timeoutConstraint, brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart)
    // Tune these by watching path-following behavior; not from a dedicated tuner OpMode.
    //
    // NOTE (Predictive Braking): once the above is tuned, consider lowering the first
    // value (parametric end / tValueConstraint) from 0.99 to something like 0.95-0.97.
    // PIDF-based following tends to overshoot and hit the parametric end early, but
    // Predictive Braking actually comes to a full stop in time, so a high value here
    // just delays when path-end actions can trigger. Don't go below ~0.9 or the path
    // will be marked complete before braking finishes.
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("lfmotor")
            .leftRearMotorName("lbmotor")
            .rightFrontMotorName("rfmotor")
            .rightRearMotorName("rbmotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            // SOURCE: Tuning OpMode -> Automatic -> Forward Velocity Tuner.
            // Robot drives forward at full power over a set distance; telemetry reports
            // "Velocity" -> copy here.
            .xVelocity(79.53968582754061)

            // SOURCE: Tuning OpMode -> Automatic -> Lateral Velocity Tuner.
            // Same as above but strafing.
            .yVelocity(57.234300027682096);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            // SOURCE: Physically measure with a ruler/tape measure -- distance (inches) from
            // robot's center of rotation to the forward pod, along the Y axis.
            .forwardPodY(-3.284)
            // SOURCE: Physically measure with a ruler/tape measure -- distance (inches) from
            // robot's center of rotation to the strafe pod, along the X axis.
            .strafePodX(-5.1715)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .yawScalar(1.0)
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}