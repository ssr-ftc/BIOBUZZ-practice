package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;

@Autonomous(name = "Half Auto 3 Tip", group = "Autonomous")
public class HalfAuto3tip extends OpMode {
    private final PoseFactory poses = PoseFactory.degrees();
    private final Pose start = poses.of(56, 9, 180);
    private final Pose gardenIntake = poses.of(24, 9, 180);
    private final Pose gardenIntakeSlow = poses.of(13.5, 9, 180);
    private final Pose shoot1 = poses.of(38, 9, 180);
    private final Pose flowerIntake = poses.of(24, 48, 180);
    private final Pose flowerIntakeSlow = poses.of(14, 48, 180);
    private final Pose shoot2 = poses.of(38, 11, 90);
    private final Pose parkPrep = poses.of(18, 95, 0);
    private final Pose end = poses.of(10, 95, 0);

    private Follower follower;
    private int pathState = 1;

    @Override
    public void init() {
        follower = Robot1.createFollower(hardwareMap);
        follower.setPose(start);
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                follower.follow(line(start, gardenIntake).linear(start, gardenIntake));
                pathState = 2;
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.follow(line(gardenIntake, gardenIntakeSlow).constant(gardenIntakeSlow));
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.follow(line(gardenIntakeSlow, shoot1).constant(shoot1));
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.follow(line(shoot1, flowerIntake).constant(flowerIntake));
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.follow(line(flowerIntake, flowerIntakeSlow).constant(flowerIntakeSlow));
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.follow(line(flowerIntakeSlow, flowerIntake).linear(flowerIntakeSlow, flowerIntake));
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.follow(line(flowerIntake, shoot2).linear(flowerIntake, shoot2));
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.follow(line(shoot2, parkPrep).linear(shoot2, parkPrep));
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.follow(line(parkPrep, end).constant(end));
                    pathState = 10;
                }
                break;
            default:
                break;
        }
    }
}
