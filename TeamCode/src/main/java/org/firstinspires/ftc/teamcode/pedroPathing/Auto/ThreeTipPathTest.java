
package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.follower.Follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Depo;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.savePosition;

@Autonomous(name = "Three tip aura farm", group = "Autonomous")
public class ThreeTipPathTest extends OpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose start = poseFactory.of(62.8888, 6.1382, 0);

    private final Pose path1 = poseFactory.of(94.9089, 13, 270);
    private final Pose path1Control1 = poseFactory.of(93.6431, 41.7943, 0);

    private final Pose point2 = poseFactory.of(46.9775, 130, 90);
    private final Pose point2Control1 = poseFactory.of(31.6205, 14.1693, 0);
    private final Pose point2Control2 = poseFactory.of(81.7501, 116.6678, 0);
    private final Pose point2Control3 = poseFactory.of(45.2218, 94.0804, 0);

    private final Pose point2Segment1Start = poseFactory.of(46.9775, 127.674, 270);
    private final Pose point2Segment1End = poseFactory.of(46.9775, 127.674, 90);
    private final Pose point2Segment3Start = poseFactory.of(46.9775, 127.674, 107.3128);
    private final Pose point2Segment3End = poseFactory.of(46.9775, 127.674, 90);
    private final Pose pointExtra = poseFactory.of(46.9775, 122, 90);


    private final Pose point3 = poseFactory.of(6.5839, 12, -90);
    private final Pose point3Control1 = poseFactory.of(9.8239, 49.7561, 0);

    private final Pose point3Segment1Start = poseFactory.of(6.5839, 13.2399, 90);
    private final Pose point3Segment1End = poseFactory.of(6.5839, 13.2399, 270);
    private final Pose point3Segment4Start = poseFactory.of(6.5839, 13.2399, -98.1388);
    private final Pose point3Segment4End = poseFactory.of(6.5839, 13.2399, 270);

    private final Pose point4Start = poseFactory.of(6.5839, 13.2399, 270);
    private final Pose point4 = poseFactory.of(39.4152, 9.8093, 180);

    private final Pose point5Start = poseFactory.of(39.4152, 9.8093, 180);
    private final Pose point5 = poseFactory.of(12, 47.1105, 180);
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


    private Path opponentFlower;//to oponent flower
    private Path bottomFlower;//to back flower
    private Path extraPath; //to extra step back
    private Path garden;//to garden
    private Path gardenToShoot;//to shoot garden balls
    private Path lastFlower;//to last flower
    private Path lastShot;//to shoot flower balls
    private Path park;//park
    private ElapsedTime didyTimer;

    private int pathState = 1;

    @Override
    public void init() {

        follower = Robot1.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        depo = new Depo(hardwareMap);
        turret = new Turret(hardwareMap);
        timers = new Timer();
        didyTimer = new ElapsedTime();

        intake.start();
        depo.start();
        turret.start();
        turret.resetTurretEncoders();
        turret.setPositionDeg(firstShotAngle);

        timers.createNew("depo");
        timers.createNew("flower");
        timers.createNew("intake");

        // Path 1
        opponentFlower = Paths.curve(start, path1Control1, path1)
                .linear(start, path1);

        // Path 2
        bottomFlower = Paths.curve(
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
        extraPath = Paths.line(point2,pointExtra).heading(Interpolator.constant(pointExtra));

        // Path 3
        garden = Paths.curve(pointExtra, point3Control1, point3)
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
        gardenToShoot = Paths.line(point4Start, point4)
                .linear(point4Start, point4);

        // Path 5
        lastFlower = Paths.curve(point5Start, point5Control1, point5)
                .linear(point5Start, point5);

        // Path 6
        lastShot = Paths.curve(point5, point6Control1, point6)
                .reverseTangent();

        // Path 7
        park = Paths.line(point6, point7)
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
    double backShotVelo=1480;
    double gardenShotVelo=1520;


    double firstShotAngle=-97;
    double bottomFlowerAngle = -192;
    double bottomFlowerAngle2 = -197;
    double gardenShotAngle = -245;
    double lastShotAngle = 37;

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
    @Override
    public void stop(){
        savePosition.endPose=follower.pose();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {
            case 1://shoot and start going to flower after
                if(depo.reachedvelocity() && !timers.isRunning("depo")){//reached velocity
                    timers.start("depo");
                }
                shootingFunc();
                if(depo.get_target_velocity()==0){//finished shooting
                    follower.follow(opponentFlower);//go to flower
                    intake.intake_on();//turn on intake
                    intake.flower_down();//lower flower intake
                    pathState = 2;
                }
                break;

            case 2://going to flower
                if (!follower.isBusy()) {//when reached stand and collect
                    didyTimer.reset();
                    pathState=21;
                }
                break;
            case 21://reached first flower, wait and collect
                if(didyTimer.seconds()>1.5){//after done waiting
                    intake.front_down();//added here to test
                    follower.follow(bottomFlower);//start dring to next flower
                    turret.setPositionDeg(bottomFlowerAngle);//change turret for the next shot
                    depo.set_target_velocity(firstShotVelo);//start the depo while driving
                    didyTimer.reset();//start the timer of finishing intaking while driving to the other side
                    pathState = 30;
                }
                break;
            case 30://robot is currently driving to next flower
                if(didyTimer.seconds()>0.7){
                    intake.front_down();
                    didyTimer.reset();
                    pathState = 301;
                }
                break;
            case 301:
                if(didyTimer.seconds()>1.8){
                    intake.intake_off();
                    intake.front_up();
                    pathState=3;
                }
                break;

            case 3://still driving but done with intake stuff from old flower
                if (!follower.isBusy()) {//when reached position open the hatch and start shooting
                    didyTimer.reset();//start timing the long shot
                    pathState=310;
                }
                break;
            case 310:
                if(didyTimer.seconds()>0.7){
                    depo.open_hatch();
                    intake.intake_on();
                    didyTimer.reset();
                    pathState = 31;
                }
                break;
            case 31:
                if(didyTimer.seconds()>1.5){
                    intake.front_down();//lower the intake again
                    depo.close_hatch();
                    depo.set_target_velocity(backShotVelo);
                    turret.setPositionDeg(bottomFlowerAngle2);

                    follower.follow(extraPath);
                    depo.setHood(0.46);
                    pathState = 312;
                }
                break;
            case 312:
                if(!follower.isBusy()){
                    didyTimer.reset();
                    depo.open_hatch();
                    pathState = 32;
                }
                break;
            case 32:
                if (didyTimer.seconds()>1.5) {//done shooting
                    depo.close_hatch();
                    depo.turn_off_deposit();
                    intake.front_up();
                    intake.flower_up();
                    follower.follow(garden);//go to garden
                    didyTimer.reset();
                    pathState = 40;
                }
                break;
            case 40://currently driving to garden
                if(didyTimer.seconds()>2){
                    intake.flower_down();
                    depo.setHood(0.44);
                    pathState = 4;
                }
                break;

            case 4://still driving to garden
                if (!follower.isBusy()) {//reached garden, wait to intake
                    didyTimer.reset();
                    pathState = 41;
                }
                break;
            case 41:
                if(didyTimer.seconds()>1){//finished intaking
                    intake.front_down();
                    follower.follow(gardenToShoot);//drive to shoot
                    depo.set_target_velocity(gardenShotVelo);//prep depo and turret for shooting
                    turret.setPositionDeg(gardenShotAngle);
                    pathState = 5;
                }
                break;

            case 5://driving to shot pose
                if (!follower.isBusy()) {//reached the shot pos
                    depo.open_hatch();
                    didyTimer.reset();
                    pathState = 51;
                }
                break;
            case 51://waiting to finish shooting
                if(didyTimer.seconds()>1.5){//done shooting
                    intake.front_up();
                    depo.turn_off_deposit();
                    depo.close_hatch();
                    depo.setHood(0.46);
                    follower.follow(lastFlower);//start driving to the last flower
                    pathState = 60;
                }
                break;
            case 60:
                if (!follower.isBusy()) {//reached the last flower
                    didyTimer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                if (didyTimer.seconds()>1.5) {//waited to intake
                    follower.follow(lastShot);
                    intake.front_down();
                    depo.set_target_velocity(gardenShotVelo);
                    turret.setPositionDeg(lastShotAngle);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()&&depo.reachedvelocity()) {
                    depo.open_hatch();
                    didyTimer.reset();
                    pathState = 71;
                }
                break;
            case 71:
                if(didyTimer.seconds()>1.5){//done shooting
                    intake.front_up();
                    depo.turn_off_deposit();
                    depo.close_hatch();
                    intake.intake_off();
                    follower.follow(park);//park
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
