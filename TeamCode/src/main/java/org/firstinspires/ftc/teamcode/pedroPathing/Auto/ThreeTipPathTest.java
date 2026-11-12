
package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.follower.Follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Depo;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;

@Autonomous(name = "Three Tip Path Test", group = "Autonomous")
public class ThreeTipPathTest extends OpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose start = poseFactory.of(62.8888, 6.1382, 0);

    private final Pose path1 = poseFactory.of(94.9089, 15.4517, 270);
    private final Pose path1Control1 = poseFactory.of(93.6431, 41.7943, 0);

    private final Pose point2 = poseFactory.of(46.9775, 127.674, 90);
    private final Pose point2Control1 = poseFactory.of(31.6205, 14.1693, 0);
    private final Pose point2Control2 = poseFactory.of(81.7501, 116.6678, 0);
    private final Pose point2Control3 = poseFactory.of(45.2218, 94.0804, 0);

    private final Pose point2Segment1Start = poseFactory.of(46.9775, 127.674, 270);
    private final Pose point2Segment1End = poseFactory.of(46.9775, 127.674, 90);
    private final Pose point2Segment3Start = poseFactory.of(46.9775, 127.674, 107.3128);
    private final Pose point2Segment3End = poseFactory.of(46.9775, 127.674, 90);

    private final Pose point3 = poseFactory.of(6.5839, 13.2399, -90);
    private final Pose point3Control1 = poseFactory.of(9.8239, 49.7561, 0);

    private final Pose point3Segment1Start = poseFactory.of(6.5839, 13.2399, 90);
    private final Pose point3Segment1End = poseFactory.of(6.5839, 13.2399, 270);
    private final Pose point3Segment4Start = poseFactory.of(6.5839, 13.2399, -98.1388);
    private final Pose point3Segment4End = poseFactory.of(6.5839, 13.2399, 270);

    private final Pose point4Start = poseFactory.of(6.5839, 13.2399, 270);
    private final Pose point4 = poseFactory.of(39.4152, 9.8093, 180);

    private final Pose point5Start = poseFactory.of(39.4152, 9.8093, 180);
    private final Pose point5 = poseFactory.of(14.0346, 47.1105, 180);
    private final Pose point5Control1 = poseFactory.of(35.2228, 49.6518, 0);

    private final Pose point6 = poseFactory.of(42.9042, 14.1248, 104.4076);
    private final Pose point6Control1 = poseFactory.of(34.0614, 48.5461, 0);

    private final Pose point7 = poseFactory.of(8.7169, 94.6822, 112.9956);

    // Follower and paths
    private Follower follower;
    Intake intake;
    Depo depo;
    Turret turret;
    Timer timers;


    private Path pathOne;//to oponent flower
    private Path pathTwo;//to back flower
    private Path pathThree;//to garden
    private Path pathFour;//to shoot garden balls
    private Path pathFive;//to last flower
    private Path pathSix;//to shoot flower balls
    private Path pathSeven;//park

    private int pathState = 1;

    @Override
    public void init() {

        follower = Robot1.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        depo = new Depo(hardwareMap);
        turret = new Turret(hardwareMap);
        timers = new Timer();

        intake.start();
        depo.start();
        turret.start();
        turret.resetTurretEncoders();
        turret.setPositionDeg(firstShotAngle);

        timers.createNew("depo");
        timers.createNew("flower");

        // Path 1
        pathOne = Paths.curve(start, path1Control1, path1)
                .linear(start, path1);

        // Path 2
        pathTwo = Paths.curve(
                path1,
                point2Control1,
                point2Control2,
                point2Control3,
                point2
        ).heading(
                Interpolator.piecewise()
                        .until(0.2419, Interpolator.linear(
                                point2Segment1Start,
                                point2Segment1End
                        ))
                        .until(0.9143, Interpolator.tangent)
                        .until(1, Interpolator.linear(
                                point2Segment3Start,
                                point2Segment3End
                        ))
        );

        // Path 3
        pathThree = Paths.curve(point2, point3Control1, point3)
                .heading(
                        Interpolator.piecewise()
                                .until(0.3573, Interpolator.linear(
                                        point3Segment1Start,
                                        point3Segment1End
                                ))
                                .until(0.8928, Interpolator.tangent)
                                .until(0.9555, Interpolator.tangent)
                                .until(1, Interpolator.linear(
                                        point3Segment4Start,
                                        point3Segment4End
                                ))
                );

        // Path 4
        pathFour = Paths.line(point4Start, point4)
                .linear(point4Start, point4);

        // Path 5
        pathFive = Paths.curve(point5Start, point5Control1, point5)
                .linear(point5Start, point5);

        // Path 6
        pathSix = Paths.curve(point5, point6Control1, point6)
                .reverseTangent();

        // Path 7
        pathSeven = Paths.line(point6, point7)
                .tangent();

        // Set starting pose
        follower.setPose(start);
        follower.update();
    }
    @Override
    public void init_loop(){
        turret.run_using_pid();
    }
    double firstShotVelo=1500;
    double firstShotAngle=-95;

    @Override
    public void start(){
        depo.set_target_velocity(firstShotVelo);
    }

    @Override
    public void loop() {

        follower.update();
        turret.run_using_pid();
        depo.run_using_pid();

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Is Busy", follower.isBusy());

        telemetry.update();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {
            case 1:
                if(depo.reachedvelocity() && !timers.isRunning("depo")){
                    timers.start("depo");
                }
                shootingFunc();
                if(depo.get_target_velocity()==0){
                    follower.follow(pathOne);
                    intake.intake_on();
                    intake.flower_down();
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {

                    timers.start("flower");
                    pathState=21;
                }
                break;
            case 21:
                if(timers.checkSecondsLast("flower",1)){
                    follower.follow(pathTwo);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.follow(pathThree);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.follow(pathFour);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.follow(pathFive);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.follow(pathSix);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.follow(pathSeven);
                    pathState = 8;
                }
                break;

            default:
                break;
        }

    }
    void shootingFunc(){

        if(timers.checkSeconds("depo",0.3)){
            intake.intake_on();
            depo.open_hatch();
        }
        if(timers.checkSecondsLast("depo", 1.5)){
            depo.close_hatch();
            depo.set_target_velocity(0);
        }
    }
}
