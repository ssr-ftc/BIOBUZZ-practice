package org.firstinspires.ftc.teamcode.OFSB2.Auto;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
/**
 * Constants for the Off Season Bot 1 (OFSWB).
 */
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(7.2)
            .forwardZeroPowerAcceleration(-39.370416251310814)
            .lateralZeroPowerAcceleration(-56.98433031510037)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.16, 0, 0.004, 0.031))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.005, 0.019))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.5, 0.031))
            .centripetalScaling(0.0005)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("rr")
            .leftRearMotorName("rl ")
            .leftFrontMotorName("fl" )
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(95.27855676365651)
            .yVelocity(73.45713601900836);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.5)
            .strafePodX(7)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("imu")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints
            (0.99, 100, 1.3, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
/*
Translational PIDF Values:
D: 0.004
F: 0.031
I: 0
P: 0.16

Heading PIDF Values
D: 0.005
F: 0.019
I: 0
P: 0.7

Drive PIDF Values
D: 0.00001
F: 0.031
I: 0
P: 0.025
T: 0.5

*/
