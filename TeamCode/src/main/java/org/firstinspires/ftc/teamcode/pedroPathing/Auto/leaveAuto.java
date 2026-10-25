package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Leave Auto", group = "Autonomous")
public class leaveAuto extends OpMode {
    private Follower follower;
    private final Pose start = new Pose(8.7297, 132.7066, Math.toRadians(90));
    private final Pose path1Start = new Pose(8.7297, 132.7066, Math.toRadians(0));
    private final Pose path1 = new Pose(8.6385, 112.5088, Math.toRadians(0));
    private PathChain leave;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(start);
        leave = follower.pathBuilder()
                .addPath(new BezierLine(path1Start, path1))
                .setLinearHeadingInterpolation(path1Start.getHeading(), path1.getHeading())
                .build();
    }

    @Override
    public void start() {
        follower.followPath(leave, true);
    }

    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            telemetry.addLine("left the start");
        }
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
