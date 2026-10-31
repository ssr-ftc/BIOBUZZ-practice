package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "leave auto blue", group = "Pedro")
public class leaveAutoBlue extends OpMode {

    private Follower follower;
    private PathChain leave;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    private final Pose endPose = new Pose(124.69239373601792, 26.819910514541387, Math.toRadians(180));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);

        leave = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void start() {
        follower.followPath(leave);
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading",
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}