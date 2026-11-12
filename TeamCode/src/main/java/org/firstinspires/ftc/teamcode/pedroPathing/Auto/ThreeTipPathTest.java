package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

@Autonomous(name = "3 tip pathing", group = "Autonomous")
public class ThreeTipPathTest extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(62.8888, 6.1382, 0);
    private final Pose path1 = poseFactory.of(94.7263, 11.9822, 270);
    private final Pose path1Control1 = poseFactory.of(93.6431, 41.7943, 0);
    private final Pose point2 = poseFactory.of(46.9775, 127.674, 90);
    private final Pose point2Control1 = poseFactory.of(31.6205, 14.1693, 0);
    private final Pose point2Control2 = poseFactory.of(81.7501, 116.6678, 0);
    private final Pose point2Control3 = poseFactory.of(45.2218, 94.0804, 0);
    private final Pose point2Segment1Start = poseFactory.of(46.9775, 127.674, 270);
    private final Pose point2Segment1End = poseFactory.of(46.9775, 127.674, 90);
    private final Pose point2Segment3Start = poseFactory.of(46.9775, 127.674, 107.6615);
    private final Pose point2Segment3End = poseFactory.of(46.9775, 127.674, 90);
    private final Pose point3 = poseFactory.of(6.036, 7.9444, -90);
    private final Pose point3Control1 = poseFactory.of(9.8239, 49.7561, 0);
    private final Pose point3Segment1Start = poseFactory.of(6.036, 7.9444, 90);
    private final Pose point3Segment1End = poseFactory.of(6.036, 7.9444, 270);
    private final Pose point3Segment4Start = poseFactory.of(6.036, 7.9444, -97.6926);
    private final Pose point3Segment4End = poseFactory.of(6.036, 7.9444, 270);
    private final Pose point4Start = poseFactory.of(6.036, 7.9444, 270);
    private final Pose point4 = poseFactory.of(39.4152, 9.8093, 180);
    private final Pose point5Start = poseFactory.of(39.4152, 9.8093, 180);
    private final Pose point5 = poseFactory.of(14.0346, 47.1105, 180);
    private final Pose point5Control1 = poseFactory.of(35.2228, 49.6518, 0);
    private final Pose point6 = poseFactory.of(42.9042, 14.1248, 104.4076);
    private final Pose point6Control1 = poseFactory.of(34.0614, 48.5461, 0);
    private final Pose point7 = poseFactory.of(8.7169, 94.6822, 112.9956);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
            follow(follower, path1()),
            follow(follower, path2()),
            follow(follower, path3()),
            follow(follower, path4()),
            follow(follower, path5()),
            follow(follower, path6()),
            follow(follower, path7())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2Control2, point2Control3, point2).heading(Interpolator.piecewise().until(0.2419, Interpolator.linear(point2Segment1Start, point2Segment1End)).until(0.9143, Interpolator.tangent).until(1, Interpolator.linear(point2Segment3Start, point2Segment3End)));
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3).heading(Interpolator.piecewise().until(0.3573, Interpolator.linear(point3Segment1Start, point3Segment1End)).until(0.8928, Interpolator.tangent).until(0.9555, Interpolator.tangent).until(1, Interpolator.linear(point3Segment4Start, point3Segment4End)));
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.curve(point5, point6Control1, point6).reverseTangent();
    }

    public Path path7() {
        return Paths.line(point6, point7).tangent();
    }
}
