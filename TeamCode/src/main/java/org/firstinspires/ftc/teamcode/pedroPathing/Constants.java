package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.pedropathing.ftc.drivetrains.MecanumConstants;

//constants for lm0 bot

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.99)
            .forwardZeroPowerAcceleration(-58.962416251310814)
            .lateralZeroPowerAcceleration(-56.99523031510037)
            .headingPIDFCoefficients(new PIDFCoefficients(0.75,0,0.0275,0.01))
            //.headingPIDFCoefficients(new PIDFCoefficients(0.44, 0, 0, 0.02))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.076, 0, 0.005, 0.027));
    //.translationalPIDFCoefficients(new PIDFCoefficients(0.04, 0, 0.004, 0.027));
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf") //fr
            .rightRearMotorName("rb")
            .leftRearMotorName("bl")  //rl
            .leftFrontMotorName("fl" ) //fl
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD) //REVERSE
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD) //REVERSE
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE) //FORWARD
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE) //FORWARD
            .useBrakeModeInTeleOp(true)
            .xVelocity(73.44555676365651)
            .yVelocity(54.21013601900836);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.5)
            .strafePodX(4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint") //"imu"
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}