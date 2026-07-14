package org.firstinspires.ftc.teamcode.OFSB1.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OFSB1.Constants;
import org.firstinspires.ftc.teamcode.OFSB1.Subsystems.OFSB1Subsystem;

/**
 * Basic Autonomous template for Off Season Bot 1 (OFSB1).
 */
@Autonomous(name = "OFSB1 Auto", group = "OFSB1")
@Disabled
public class OFSB1Auto extends OpMode {

    private Follower follower;
    private OFSB1Subsystem robot;

    private Path testPath;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        robot = new OFSB1Subsystem(hardwareMap);

        // Create a simple test path: move forward 24 inches
        testPath = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(24, 0, 0)));
        testPath.setConstantHeadingInterpolation(0);

        telemetry.addData("Status", "OFSB1 Auto Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(testPath);
    }

    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy()) {
            telemetry.addLine("Path Complete!");
        }

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stopAll();
    }
}
