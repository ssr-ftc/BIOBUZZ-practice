package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import static com.pedropathing.api.Paths.line;

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

    private final Pose leftReset = new Pose(81, 9, Math.toRadians(90));
    private final Pose leftTarget = new Pose(79, 12, Math.toRadians(88.5));
    private final Pose leftMiddle = new Pose(79, 28, Math.toRadians(90));

    private final Pose rightTarget = new Pose(58, 12, Math.toRadians(90));
    private final Pose rightReset = new Pose(58, 9, Math.toRadians(90));
    private final Pose rightMiddle = new Pose(58, 28, Math.toRadians(90));
    public boolean rightSide = false;


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
        if(gamepad2.dpadRightWasPressed()){
            de.setHood(de.getHood()+0.005);
        }
        if(gamepad2.dpadLeftWasPressed()){
            de.setHood(de.getHood()-0.005);
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
        perfectShot();
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

    public void perfectShot() {
        if (gamepad1.dpadLeftWasPressed()) {
            follower.setPose(leftReset);
            rightSide = false;
        }
        if (gamepad1.dpadRightWasPressed()) {
            follower.setPose(rightReset);
            rightSide = true;
        }
        if (gamepad1.squareWasPressed()) {
            if (rightSide) {
                follower.follow(line(follower.pose(), rightMiddle).linear(follower.pose().heading(), rightMiddle.heading()));
                follower.follow(line(rightMiddle, rightTarget).linear(rightMiddle.heading(), rightTarget.heading()));
            } else {
                follower.follow(line(follower.pose(), leftMiddle).linear(follower.pose().heading(), leftMiddle.heading()));
                follower.follow(line(leftMiddle, leftTarget).linear(leftMiddle.heading(), leftTarget.heading()));
            }
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
        if (follower.isBusy()) {
            return;
        }
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
        telemetry.addData("Side", rightSide ? "RIGHT" : "LEFT");
        telemetry.addData("Robot Pose", follower.pose().toString());
        telemetry.addLine("");
        telemetry.addLine("");
        telemetry.addLine("     -----  CONTROLS  -----     ");
        telemetry.addLine("Base driver:");
        telemetry.addLine("Left Joystick: Move and Strafe");
        telemetry.addLine("Right Joystick: Rotate");
        telemetry.addLine("Right Trigger: Shift Speed");
        telemetry.addLine("Y:  Flower up/down");
        telemetry.addLine("X: Auto tleeop localize");
        telemetry.addLine("Dpad left: Left localize");
        telemetry.addLine("Dpad right: Right Localize");
        telemetry.addLine("");
        telemetry.addLine("Shoter driver:");
        telemetry.addLine("Right Bymper: INtake");
        telemetry.addLine("Left Bumper: Outtake");
        telemetry.addLine("A: SHoot");
        telemetry.addLine("Dpad up/down:  velocity +- 50");
        telemetry.addLine("Dpad right: hoodup");
        telemetry.addLine("Dpad right: hoodown");
        telemetry.addLine("");
        telemetry.addLine("please reference drivers if controls do not work");
        telemetry.update();
    }
}
