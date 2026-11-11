//package org.firstinspires.ftc.teamcode.pedroPathing.Tests;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Depo;
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Transfer;
//
//@TeleOp(name = "Intake & Transfer Test", group = "tuning")
//    @Config
//    public class IntakeTransferTest extends LinearOpMode {
//        Intake intake;
//        Transfer transfer;
//        Depo depo;
//        boolean useTelem;
//        private enum mode {telem,NoTelem};
//        private mode m = mode.NoTelem;
//        boolean backState, frontState;
//        double backBalls, frontBalls, totalBalls;
//
//
//        private FtcDashboard dashboard = FtcDashboard.getInstance();
//
//        @Override
//        public void runOpMode() {
//            intake = new Intake(hardwareMap);
//            depo = new Depo(hardwareMap);
//            transfer = new Transfer(hardwareMap);
//
//            Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
//
//            waitForStart();
//            intake.start();
//            depo.close_hatch();
//            while (opModeIsActive()) {
//                if(gamepad1.rightBumperWasPressed()){
//                    if(intake.intaking()) intake.intake_off();
//                    else intake.intake_on();
//                }
//                if(gamepad1.left_bumper){
//                    intake.outtake();
//                }
//                else if(!intake.intaking()){
//                    intake.intake_off();
//                }
//                countBalls();
//                totalBalls=frontBalls+backBalls;
//                if(intake.intaking()){
//                    if(totalBalls>=4){
//                        intake.front_up();
//                        intake.back_up();
//                        intake.intake_off();
//                    }
//                }
//
//
//
//                if(gamepad1.cross){
//                    transfer.setFlap_mid();
//                }if(gamepad1.circle){
//                    transfer.setFlap_back();
//                }if(gamepad1.square){
//                    transfer.setFlap_front();
//                }
//
//                if(gamepad1.dpad_up){
//                    intake.front_up();
//                    intake.back_up();
//                }
//                if(gamepad1.dpadDownWasPressed()){
//                    intake.front_down();
//                    intake.back_down();
//                    backBalls=0;
//                    frontBalls=0;
//                }
//                if(gamepad1.triangleWasPressed()){
//                    useTelem = !useTelem;
//                }
//                if(useTelem){
//                    telemetry.addData("front sensor dist", transfer.getDistanceFront_MM());
//                    telemetry.addData("back sensor dist", transfer.getDistanceBack_MM());
//                }
//                telemetry.addData("back balls",backBalls);
//                telemetry.addData("front balls",frontBalls);
//                telemetry.addData("total balls",totalBalls);
//                telemetry.update();
//            }
//        }
//        public void countBalls(){
//            countBack();
//            countFront();
//        }
//        public void countBack(){
//            if(transfer.getDistanceBack_MM()<65 && !backState) backState=true;
//            if(backState){
//                if(transfer.getDistanceBack_MM()>80){
//                    backBalls+=1;
//                    backState=false;
//                }
//            }
//        }
//    public void countFront(){
//        if(transfer.getDistanceFront_MM()<65 && !frontState) frontState=true;
//        if(frontState){
//            if(transfer.getDistanceFront_MM()>80){
//                frontBalls+=1;
//                frontState=false;
//            }
//        }
//    }
//    }
//
////hiiiiii
