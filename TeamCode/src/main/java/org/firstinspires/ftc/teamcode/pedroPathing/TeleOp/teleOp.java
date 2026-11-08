package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot1;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Depo;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Transfer;


@TeleOp(name = "LM1 TeleOp", group = "Templates")
public class teleOp extends OpMode {

    private Follower follower;//this is pedropathing follower
    Depo de;
    Intake intake;
    Transfer transfer;
    Timer timers;
    double velo = 1500;
    private double speedScale = 0.9;
    boolean backState,frontState;
    double backBalls,frontBalls;
    private boolean IntakeAutoStopped = false;
    private boolean manualOverride = false;


    @Override
    public void init() {//this is called once when the driver presses init
        follower = Robot1.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));
        intake = new Intake(hardwareMap);
        de = new Depo(hardwareMap);
        transfer = new Transfer(hardwareMap);
        timers = new Timer();
        telemetry.addLine("Initialized");
        telemetry.addData("Status", "Press start to run TeleOp");
        telemetry.addData("Starting Pose", "(%.1f, %.1f, %.1f)", 0.0, 0.0, 0.0);
        telemetry.update();
    }


    @Override
    public void start() {//this is called once when the driver presses start
        de.start();
        intake.start();
        //transfer.start();
        timers.createNew("depo");
    }

    @Override
    public void loop() {//this is called repeatedly during the op mode
        de.run_using_pid();

        if(gamepad2.dpadUpWasPressed()){
            velo+=50;
        }if(gamepad2.dpadDownWasPressed()){
            velo-=50;
        }

        if(gamepad2.rightBumperWasPressed()){
            if (intake.intaking()) {
                intake.front_up();
                intake.back_up();
                intake.intake_off();
                IntakeAutoStopped = false;
                manualOverride = false;
            } else {
                intake.front_down();
                intake.back_down();
                intake.intake_on();
                IntakeAutoStopped = false;
                if ((frontBalls + backBalls) >= 4) {
                    manualOverride = true;
                }
            }
        }

        if (gamepad2.leftBumperWasPressed()) {
            if (intake.outtaking()) {
                intake.intake_off();
            } else {
                intake.outtake();
            }
        }

        if(!de.isDepositOn()){
            countBalls();
        }
        if ((frontBalls + backBalls) < 4) {
            IntakeAutoStopped = false;
            manualOverride = false;
        }
        if(intake.intaking() && !de.isDepositOn() && !IntakeAutoStopped && !manualOverride){
            if((frontBalls+backBalls)>=4){
                intake.front_up();
                intake.back_up();
                intake.intake_off();
                IntakeAutoStopped = true;
            }
        }
        if(gamepad2.cross && !de.isDepositOn()){
            de.set_target_velocity(velo);
        }

        shootingFunc();
        flower();
        driveWithPedro();
        sendTelemetry();
        follower.update();
    }
    void shootingFunc(){
        if(de.reachedvelocity() && !timers.isRunning("depo")){
            timers.start("depo");
        }
        if(timers.checkSeconds("depo",0.3)){
            intake.intake_on();
            de.open_hatch();
        }
        if(timers.checkSecondsLast("depo", 2.5)){
            de.close_hatch();
            de.set_target_velocity(0);
            intake.intake_off();
            /*
            intake.front_down();
            intake.back_down();

             */
            frontBalls=0;
            backBalls=0;
        }
    }

    public void countBalls(){
        countBack();
        countFront();
    }
    public void countBack(){
        if(intake.getDistanceBack_MM()<65 && !backState) backState=true;
        if(backState){
            if(intake.getDistanceBack_MM()>80){
                backBalls+=1;
                backState=false;
            }
        }
    }
    public void countFront(){
        if(intake.getDistanceFront_MM()<65 && !frontState) frontState=true;
        if(frontState){
            if(intake.getDistanceFront_MM()>80){
                frontBalls+=1;
                frontState=false;
            }
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
        double strafe = -gamepad1.left_stick_x  * speedScale;
        double turn = -gamepad1.right_stick_x * speedScale;

        follower.manual(forward, strafe, turn);
    }

    private void flower() {
        if (gamepad1.triangleWasPressed()) {
            if (intake.isFlowerDown()) {
                intake.flower_up();
            } else {
                intake.front_up();
                intake.flower_down();
            }
        }
    }



    private void sendTelemetry() {
        telemetry.addData("velo",velo);
        telemetry.addData("depo velo",de.getVelocity());
        telemetry.addData("total balls",frontBalls+backBalls);
        telemetry.update();
    }
}