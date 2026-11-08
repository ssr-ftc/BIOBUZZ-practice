package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Depo;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;


@TeleOp(name = "Solo TeleOp", group = "Templates")
public class soloTeleOp extends OpMode {

    private Follower follower;//this is pedropathing follower
    Pose localGarden = new Pose(64.5, 8.5, Math.toRadians(90));
    Pose localFlower = new Pose(64.5,133.5, Math.toRadians(270));
    Depo depo;
    Intake intake;
    Turret turret;
    Timer timers;
    double distanceToGoal;
    boolean useTurret=true;
    boolean autoTracking = false;
    double velo = 1500;
    private double speedScale = 1;
    private boolean IntakeAutoStopped = false;
    private boolean manualOverride = false;


    @Override
    public void init() {//this is called once when the driver presses init
        follower = Robot1.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));
        intake = new Intake(hardwareMap);
        depo = new Depo(hardwareMap);
        turret = new Turret(hardwareMap);
        timers = new Timer();
        telemetry.addLine("Initialized");
        telemetry.addData("Status", "Press start to run TeleOp");
        telemetry.addData("Starting Pose", "(%.1f, %.1f, %.1f)", 0.0, 0.0, 0.0);
        telemetry.update();
    }


    @Override
    public void start() {//this is called once when the driver presses start
        depo.start();
        intake.start();
        timers.createNew("depo");
        turret.setPositionDeg(0);
    }

    @Override
    public void loop() {//this is called repeatedly during the op mode
        depo.run_using_pid();
        if(gamepad1.shareWasPressed()){
            useTurret = !useTurret;
        }
        if(gamepad1.psWasPressed()){
            turret.resetTurretEncoders();
        }
        if(gamepad1.circleWasPressed()){
            follower.setPose(localGarden);
            autoTracking = true;
        }
        if(gamepad1.squareWasPressed()){
            follower.setPose(localFlower);
            autoTracking = true;
        }
        if(autoTracking){
            if(follower.pose().y()>71){
                turret.faceFlowerHive(follower.pose());
            }
            else {
                turret.faceGardenHive(follower.pose());
            }
        }
        if(useTurret){
            turret.run_using_pid();
        }
        else turret.turretOff();


        if(gamepad1.dpadUpWasPressed()){
            velo+=25;
        }if(gamepad1.dpadDownWasPressed()){
            velo-=25;
        }
        if(gamepad1.dpadRightWasPressed()){
            depo.setHood(depo.getHood()+0.005);
        }
        if(gamepad1.dpadLeftWasPressed()){
            depo.setHood(depo.getHood()-0.005);
        }
        distanceToGoal = (follower.pose().y()>71)? turret.distanceToFlowerHive(follower.pose()) : turret.distanceToGardenHive(follower.pose());

        // Right Bumper logic with flower dependency
        if(gamepad1.rightBumperWasPressed()){
            if (intake.intaking()) {
                intake.front_up();
                intake.back_up();
                intake.intake_off();
                IntakeAutoStopped = false;
                manualOverride = false;
            } else {
                intake.intake_on();

                if (!intake.isFlowerDown()) {
                    intake.front_down();
                    intake.back_down();
                }

                IntakeAutoStopped = false;
                if ((intake.getBalls()) >= 4) {
                    manualOverride = true;
                }
            }
        }

        if (gamepad1.leftBumperWasPressed()) {
            if (intake.outtaking()) {
                intake.intake_off();
            } else {
                intake.outtake();
            }
        }

        if(!depo.isDepositOn()){
            intake.countBalls();
        }
        if ((intake.getBalls()) < 4) {
            IntakeAutoStopped = false;
            manualOverride = false;
        }
        if(intake.intaking() && !depo.isDepositOn() && !IntakeAutoStopped && !manualOverride){
            if((intake.getBalls())>=4){
                intake.front_up();
                intake.back_up();
                intake.intake_off();
                IntakeAutoStopped = true;
            }
        }
        if(gamepad1.cross && !depo.isDepositOn()){
            depo.set_target_velocity(velo);
        }

        shootingFunc();
        flower();
        driveWithPedro();
        sendTelemetry();
        follower.update();
    }

    void shootingFunc(){
        if(depo.reachedvelocity() && !timers.isRunning("depo")){
            timers.start("depo");
        }
        if(timers.checkSeconds("depo",0.3)){
            depo.open_hatch();
        }
        if(timers.checkSeconds("depo", 0.6)){
            intake.intake_on();
        }
        if(timers.checkSecondsLast("depo", 2.5)){
            depo.close_hatch();
            depo.set_target_velocity(0);
            intake.intake_off();
            intake.resetBalls();
        }
    }



    private void driveWithPedro() {
        if (gamepad1.right_trigger > 0.1) {
            speedScale = 0.25;
            gamepad1.rumbleBlips(1);
        } else {
            speedScale = 0.9;
        }

        double forward = -gamepad1.left_stick_y * speedScale;
        double strafe = -gamepad1.left_stick_x * speedScale;
        double turn = -gamepad1.right_stick_x * speedScale;

        follower.manual(forward, strafe, turn);
    }

    private void flower() {
        if (gamepad1.triangleWasPressed()) {
            if (intake.isFlowerDown()) {
                intake.flower_up();
            } else {
                intake.flower_down();
            }
        }
    }

    private void sendTelemetry() {
        telemetry.addData("hood pos", depo.getHood());
        telemetry.addData("distance to goal", distanceToGoal);
        telemetry.addData("velo",velo);
        telemetry.addData("depo velo", depo.getVelocity());
        telemetry.addData("total balls",intake.getBalls());
        telemetry.addData("turret running",useTurret);
        telemetry.addData("auto tracking",autoTracking);
        telemetry.addData("turret position",turret.getPositionDeg());
        telemetry.addData("target position",turret.getTargetDeg());
        telemetry.addData("robot x",follower.pose().x());
        telemetry.addData("robot y",follower.pose().y());
        telemetry.addData("robot heading",Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }
}