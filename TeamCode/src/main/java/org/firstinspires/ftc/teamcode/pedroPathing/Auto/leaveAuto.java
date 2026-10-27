package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Leave Auto", group = "Autonomous")
public class leaveAuto extends OpMode {
    private Follower follower;
    private final Pose start = Constants.pose(8.7297, 132.7066, Math.toRadians(90));
    private final Pose path1Start = Constants.pose(8.7297, 132.7066, Math.toRadians(0));
    private final Pose path1 = Constants.pose(8.6385, 112.5088, Math.toRadians(0));
    private Path leave;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(start);
        leave = line(path1Start, path1).linear(path1Start, path1);
    }

    @Override
    public void start() {
        follower.follow(leave);
    }

    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            telemetry.addLine("left the start");
        }
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }
}
