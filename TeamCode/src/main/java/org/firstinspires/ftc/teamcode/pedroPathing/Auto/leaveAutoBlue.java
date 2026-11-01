package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot0;

import static com.pedropathing.api.Paths.line;

@Autonomous(name = "leave auto blue", group = "Pedro")
public class leaveAutoBlue extends OpMode {

    private Follower follower;
    private Path leave;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    private final Pose endPose = new Pose(124.69239373601792, 26.819910514541387, Math.toRadians(180));

    @Override
    public void init() {
        follower = Robot0.createFollower(hardwareMap);

        follower.setPose(startPose);

        leave = line(startPose, endPose).linear(startPose, endPose);
    }

    @Override
    public void start() {
        follower.follow(leave);
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading",
                Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }
}