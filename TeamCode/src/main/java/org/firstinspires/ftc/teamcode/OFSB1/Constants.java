package org.firstinspires.ftc.teamcode.OFSB1;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
            .mass(6.216312)

            // ---- Zero Power Accelerations ----
            // SOURCE: Tuning OpMode -> Automatic -> Forward Zero Power Acceleration Tuner
            // Robot speeds up then cuts power; telemetry reports "Deceleration" -> copy here.
            .forwardZeroPowerAcceleration(-42.23391869597209)

            // SOURCE: Tuning OpMode -> Automatic -> Lateral Zero Power Acceleration Tuner
            // Same as above but strafing left/right.
            .lateralZeroPowerAcceleration(-51.97916129649106)

            // ---- Translational PID ----
            // SOURCE: FTC Dashboard / Panels, live-tuned.
            // Run StraightBackAndForth (or Line Tuner) with useHeading OFF, translational ON.
            // Manually push the robot off-line and adjust P/I/D/F in the dashboard until it
            // snaps back quickly without oscillating. Copy the final values here.
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.1, // P
                    0, // I
                    0.01, // D
                    0.03  // F
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
                    0, // D
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

            // ---- Drive PID ----
            // SOURCE: FTC Dashboard / Panels, live-tuned.
            // Run Drive Tuner OpMode (useDrive, useHeading, useTranslational all ON).
            // Start P very low (hundredths/thousandths), D even smaller. Higher = faster but
            // more overshoot at path end; lower = slower with less overshoot.
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0, // P
                    0, // I
                    0, // D
                    0, // F
                    0  // T (Kalman filter term)
            ))
            // SOURCE: Same Drive Tuner session, tuned after the primary drive PID above.
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0, // P
                    0, // I
                    0, // D
                    0, // F
                    0  // T (Kalman filter term)
            ))
            // SOURCE: Also set during drive tuning -- error distance to switch to secondary PID.
            .drivePIDFSwitch(0)

            // ---- Centripetal Scaling ----
            // SOURCE: Run a curved-path test OpMode (e.g. Circle test) AFTER all PIDs above
            // are tuned. Increase if the robot cuts corners too aggressively on curves,
            // decrease if it overcorrects/wobbles on curves.
            .centripetalScaling(0);

    // SOURCE: These 4 numbers are your path-following tolerances/limits. Confirm which
    // constructor overload your installed Pedro Pathing version uses -- current docs show:
    // (tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint,
    //  timeoutConstraint, brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart)
    // Tune these by watching path-following behavior; not from a dedicated tuner OpMode.
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            // SOURCE: Tuning OpMode -> Automatic -> Forward Velocity Tuner.
            // Robot drives forward at full power over a set distance; telemetry reports
            // "Velocity" -> copy here.
            .xVelocity(88.71476973886564)

            // SOURCE: Tuning OpMode -> Automatic -> Lateral Velocity Tuner.
            // Same as above but strafing.
            .yVelocity(74.25493183286171);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            // SOURCE: Physically measure with a ruler/tape measure -- distance (inches) from
            // robot's center of rotation to the forward pod, along the Y axis.
            .forwardPodY(-5)
            // SOURCE: Physically measure with a ruler/tape measure -- distance (inches) from
            // robot's center of rotation to the strafe pod, along the X axis.
            .strafePodX(0.5)
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