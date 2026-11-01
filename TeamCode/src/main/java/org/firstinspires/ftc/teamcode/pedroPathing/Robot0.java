package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** LM0 robot. Values are the Quickstart AutoTune. */
public class Robot0 {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("rf");
        c.backLeftName.set("bl");
        c.backRightName.set("rb");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-4.4870313509242745);
        c.yPodOffset.set(-5.453916534664124);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.4299661617584779);
        Controller secondaryTranslationalForward = Controller.proportional(0.15886100786232116);
        Controller primaryTranslationalLateral = Controller.proportional(0.5223919608506417);
        Controller secondaryTranslationalLateral = Controller.proportional(0.19300987096403951);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.014975872806358696));
        c.brake.set(Controller.proportionalFeedforward(0.01272949188540489));

        c.headingFeedback.set(Controller.proportional(4.2084463209123735));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06652736960425366, 0.00252803030910846));

        c.linearBrakeCoefficients.set(Matrix.diag(0.07475724714080365, 0.10334940627326951));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0017060378862578326, 5.819531870130576E-4));

        c.maxAchievableForwardVelocity.set(72.34490979232528);
        c.maxAchievableStrafeVelocity.set(58.4927510767619);
        c.naturalForwardDeceleration.set(45.90657422624495);
        c.naturalStrafeDeceleration.set(70.59153401642546);
    });

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
