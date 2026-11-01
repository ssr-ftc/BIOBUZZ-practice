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

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("rf");
        c.backLeftName.set("bl");
        c.backRightName.set("rb");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-5.050568467988743);
        c.yPodOffset.set(-0.44390201568603516);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3204716702903779);
                Controller secondaryTranslationalForward = Controller.proportional(0.11840571901155447);
                Controller primaryTranslationalLateral = Controller.proportional(0.45744026025905915);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1690119531368535);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012116123856994041));
                c.brake.set(Controller.proportionalFeedforward(0.010298705278444936));

                c.headingFeedback.set(Controller.proportional(4.024387452818656));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04552763680304592, 0.005262030035155689));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1077820435984096, 0.056831966817147764));
                c.quadraticBrakeCoefficients.set(Matrix.diag(8.844235263178116E-4, 0.0019525917516190305));

                c.maxAchievableForwardVelocity.set(79.88092128133736);
                c.maxAchievableStrafeVelocity.set(64.60732471003448);
                c.naturalForwardDeceleration.set(34.778181353675706);
                c.naturalStrafeDeceleration.set(61.21751646593197);
            }
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    /**
     * Pedro 3 limits path speed as a fraction of the robot's max speed.
     * 1 or more removes the limit. This replaces the old setMaxPower calls.
     */
    public static void setPathPower(double fractionOfMaxSpeed) {
        if (fractionOfMaxSpeed >= 0.999) {
            foresightConfig.maxPathSpeed.set(ForesightConfig.Constraint.NONE);
        } else {
            foresightConfig.maxPathSpeed.set(fractionOfMaxSpeed);
        }
    }
}
