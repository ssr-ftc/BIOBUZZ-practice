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


@TeleOp(name = "soloTeleop", group = "Templates")
public class soloTeleop extends OpMode {

    private Follower follower;//this is pedropathing follower
    Depo de;
    Intake intake;
    Transfer transfer;
    Timer timers;
    double velo = 1500;
    private double speedScale = 1;
    boolean backState,frontState;
    double backBalls,frontBalls;


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
        transfer.start();
        timers.createNew("depo");
    }

    @Override
    public void loop() {//this is called repeatedly during the op mode
        de.run_using_pid();

        if(gamepad1.dpadUpWasPressed()){
            velo+=50;
        }if(gamepad1.dpadDownWasPressed()){
            velo-=50;
        }

        if(gamepad1.rightBumperWasPressed()){
            intake.back_down();
            intake.front_down();
            intake.intake_on();
        }
        if(!de.isDepositOn()){
            countBalls();
        }
        if(intake.intaking() && !de.isDepositOn()){
            if((frontBalls+backBalls)>=4){
                intake.front_up();
                intake.back_up();
                intake.intake_off();
            }
        }

        if(gamepad1.cross && !de.isDepositOn()){
            de.set_target_velocity(velo);
        }

        shootingFunc();
        driveWithPedro();
        sendTelemetry();
        follower.update();
    }
    void shootingFunc(){
        if(de.reachedvelocity() && !timers.isRunning("depo")){
            timers.start("depo");
        }
        if(timers.checkSeconds("depo",0.3)){
            de.open_hatch();
            intake.intake_on();
        }
        if(timers.checkSecondsLast("depo", 2.5)){
            de.close_hatch();
            de.set_target_velocity(0);
            intake.front_down();
            intake.back_down();
            frontBalls=0;
            backBalls=0;
        }
    }

    public void countBalls(){
        countBack();
        countFront();
    }
    public void countBack(){
        if(transfer.getDistanceBack_MM()<65 && !backState) backState=true;
        if(backState){
            if(transfer.getDistanceBack_MM()>80){
                backBalls+=1;
                backState=false;
            }
        }
    }
    public void countFront(){
        if(transfer.getDistanceFront_MM()<65 && !frontState) frontState=true;
        if(frontState){
            if(transfer.getDistanceFront_MM()>80){
                frontBalls+=1;
                frontState=false;
            }
        }
    }

    private void driveWithPedro() {
        double speedShift = 0.9;

        double forward = -gamepad1.left_stick_y * speedScale;
        double strafe = -gamepad1.left_stick_x  * speedScale;
        double turn = -gamepad1.right_stick_x * speedScale;

        follower.manual(forward, strafe, turn);
    }



    private void sendTelemetry() {
        telemetry.addData("velo",velo);
        telemetry.addData("depo velo",de.getVelocity());
        telemetry.addData("total balls",frontBalls+backBalls);

        telemetry.update();
    }
}
