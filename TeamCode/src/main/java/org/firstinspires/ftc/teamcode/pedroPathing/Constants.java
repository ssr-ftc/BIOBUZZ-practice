package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Pedro Pathing 3 constants for the LM0 robot.
 * Motor names, directions, Pinpoint offsets, and measured speeds are carried over
 * from the Pedro 2 file. Foresight gains still need an AutoTune pass.
 */
public class Constants {
    private static final PoseFactory RADIANS = PoseFactory.radians();

    public static Pose pose(double x, double y, double headingRadians) {
        return RADIANS.of(x, y, headingRadians);
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
                c.frontLeftName.set("fl");
                c.backLeftName.set("bl");
                c.frontRightName.set("rf");
                c.backRightName.set("rb");

                c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.xPodOffset.set(4.0);
                c.yPodOffset.set(2.5);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller forward = Controller.proportional(0.076);
                Controller strafe = Controller.proportional(0.076);

                c.forwardTranslational.set(forward);
                c.strafeTranslational.set(strafe);

                c.coast.set(Controller.proportionalFeedforward(0.027));
                c.brake.set(Controller.proportionalFeedforward(0.027));

                c.headingFeedback.set(Controller.proportional(0.75));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.056, 0.006));

                c.linearBrakeCoefficients.set(Matrix.diag(0.106, 0.087));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015, 0.0014));

                c.maxAchievableForwardVelocity.set(73.44555676365651);
                c.maxAchievableStrafeVelocity.set(54.21013601900836);
                c.naturalForwardDeceleration.set(58.962416251310814);
                c.naturalStrafeDeceleration.set(56.99523031510037);
            }
    );

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return create(hardwareMap);
    }
}
