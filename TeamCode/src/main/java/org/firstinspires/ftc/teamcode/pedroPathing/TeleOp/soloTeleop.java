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
import org.firstinspires.ftc.teamcode.pedroPathing.Auto.EntireAuto;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Turret;


@TeleOp(name = "deandreAytonSCUKS", group = "Templates")
public class soloTeleop extends OpMode {

    private Follower follower;//this is pedropathing follower
    Depo de;
    Intake intake;
    Transfer transfer;
    Turret turret;
    Timer timers;
    double velo = 1500;
    private double speedScale = 0.9;
    boolean backState,frontState;
    double backBalls,frontBalls;
    public static boolean isBlue;
    public static boolean isRed;


    @Override
    public void init() {//this is called once when the driver presses init
        isBlue = EntireAuto.whichSide()[0];
        isRed = EntireAuto.whichSide()[1];
        follower = Robot1.createFollower(hardwareMap);
        follower.setPose(EntireAuto.returnToTeleop());
        intake = new Intake(hardwareMap);
        de = new Depo(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
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
        turret.run_using_pid();
        autoTracking();

        if(gamepad1.dpadUpWasPressed()){
            velo+=50;
        }if(gamepad1.dpadDownWasPressed()){
            velo-=50;
        }

        if(gamepad1.rightBumperWasPressed()){
            if (intake.intaking()) {
                intake.front_up();
                intake.back_up();
                intake.intake_off();
            } else {
                intake.front_down();
                intake.back_down();
                intake.intake_on();
            }
        }
        if (gamepad1.leftBumperWasPressed()) {
            if (intake.outtaking()) {
                intake.intake_off();
            } else {
                intake.outtake();
            }
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
            de.open_hatch();
            intake.intake_on();
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
                intake.flower_down();
            }
        }
    }

    public void autoTracking() {
        if (isBlue) {
            if (follower.pose().x() > 72 && follower.pose().y() < 72) {
                turret.turretAngleBlueOpposite(new Pose(follower.pose().x(), follower.pose().y(), follower.pose().heading()));
            }

            else if (follower.pose().x() > 72 && follower.pose().y() > 72) {
                turret.turretAngleBlueGarden(new Pose(follower.pose().x(), follower.pose().y(), follower.pose().heading()));
            }
        }

        else if (isRed) {
            if (follower.pose().x() < 72 && follower.pose().y() > 72) {
                turret.turretAngleRedOpposite(new Pose(follower.pose().x(), follower.pose().y(), follower.pose().heading()));
            }

            else if (follower.pose().x() < 72 && follower.pose().y() < 72) {
                turret.turretAngleRedGarden(new Pose(follower.pose().x(), follower.pose().y(), follower.pose().heading()));
            }
        }
    }



    private void sendTelemetry() {
        telemetry.addData("velo",velo);
        telemetry.addData("depo velo",de.getVelocity());
        telemetry.addData("total balls",frontBalls+backBalls);
        telemetry.addData("CURRENT TURRET ANGLE", turret.getPositionDeg());

        telemetry.update();
    }
}
