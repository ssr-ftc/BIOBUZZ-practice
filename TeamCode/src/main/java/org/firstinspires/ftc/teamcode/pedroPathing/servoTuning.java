package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.templates.TemplateSubsystem;

/**
 * Template iterative TeleOp using PedroPathing-style lifecycle methods.
 *
 * This uses init(), init_loop(), start(), loop(), and stop() instead of LinearOpMode.
 */
@TeleOp(name = "Servo tuner", group = "Templates")
public class servoTuning extends OpMode {

    //    private Follower follower;//this is pedropathing follower
//    private TemplateSubsystem subsystem;
    private double speedScale = 0.8;
    double leftDown = 0.07;
    double rightDown = 0.02;
    double backDown = 0.04;
    double leftUp = 0.40;
    double rightUp = 0.31;
    double backUp = 0.33;
//    Servo leftLever;
    private Servo[] servos = new Servo[3];
    private DcMotor intake;
    private String[] names = {"left","right","back"};
    private int cycle=0;


    @Override
    public void init() {//this is called once when the driver presses init
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(0, 0, 0));

//        subsystem = new TemplateSubsystem(hardwareMap);
//        leftLever = hardwareMap.get(Servo.class, "lift_left");
        servos[0] = hardwareMap.get(Servo.class, "lift_left");
        servos[1] = hardwareMap.get(Servo.class, "lift_right");
        servos[1].setDirection(Servo.Direction.REVERSE);
        servos[2] = hardwareMap.get(Servo.class, "lift_back");
        intake = hardwareMap.get(DcMotor.class, "intake");
//        servos[3] = hardwareMap.get(Servo.class, "servo4");
//        servos[4] = hardwareMap.get(Servo.class, "servo5");
        telemetry.addLine("Initialized");
        telemetry.addData("Status", "Press start to run TeleOp");
        telemetry.addData("Starting Pose", "(%.1f, %.1f, %.1f)", 0.0, 0.0, 0.0);
        telemetry.update();
    }

    @Override
    public void init_loop() {//this is called repeatedly during initiallzation
//        follower.update();

        telemetry.update();
    }

    @Override
    public void start() {//this is called once when the driver presses start
//        follower.startTeleopDrive();
//        leftLever.setPosition(0.2);
        allDown();
    }

    @Override
    public void loop() {//this is called repeatedly during the op mode
//        follower.update();
//        driveWithPedro();
        if(gamepad1.right_bumper){
            intake.setPower(-1);
        }else if(gamepad1.left_bumper){
            intake.setPower(1);
        }
        else intake.setPower(0);
        if(gamepad1.dpadDownWasPressed()){
            servos[cycle].setPosition(servos[cycle].getPosition()-0.01);
        }
        if(gamepad1.dpadUpWasPressed()){
            servos[cycle].setPosition(servos[cycle].getPosition()+0.01);
        }
        if(gamepad1.crossWasPressed()){
            if(cycle==2) cycle=0;
            else cycle+=1;
        }
        if(gamepad1.squareWasPressed()) {
            if (servos[0].getPosition() < 0.2) servos[0].setPosition(leftUp);
            else servos[0].setPosition(leftDown);
        }
        if(gamepad1.circleWasPressed()) {
            if (servos[1].getPosition() < 0.2) servos[1].setPosition(rightUp);
            else servos[1].setPosition(rightDown);
        }
        if(gamepad1.triangleWasPressed()) {
            if (servos[2].getPosition() < 0.2) servos[2].setPosition(backUp);
            else servos[2].setPosition(backDown);
        }
//        if(gamepad1.dpad_down)
//        sendTelemetry();
        telemetry.addData("servo name",names[cycle]);
        telemetry.addData("servo position",servos[cycle].getPosition());
    }

    @Override
    public void stop() {//this is called once when the driver presses stop
//        subsystem.stopAll();
//        follower.setTeleOpDrive(0, 0, 0, true);
//        follower.update();
    }
    private void allDown(){
        servos[0].setPosition(leftDown);
        servos[1].setPosition(rightDown);
        servos[2].setPosition(backDown);
    }

    private void driveWithPedro() {

        double forward = -gamepad1.left_stick_y * speedScale;
        double strafe = -gamepad1.left_stick_x * speedScale;
        double turn = -gamepad1.right_stick_x * speedScale;

//        follower.setTeleOpDrive(forward, strafe, turn, true);
    }



    private void sendTelemetry() {
        telemetry.addLine("this is telemetry");
        telemetry.addData("speed scale",speedScale);//example of how to use telemetry
        telemetry.update();
    }
}
