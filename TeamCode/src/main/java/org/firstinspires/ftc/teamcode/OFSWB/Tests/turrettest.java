package org.firstinspires.ftc.teamcode.OFSWB.Tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.templates.TemplateSubsystem;


@TeleOp(name = "turret test", group = "Templates")
public class turrettest extends OpMode {

    private Servo servoturret;

private TemplateSubsystem subsystem;
    @Override
    public void init() {//this is called once when the driver presses init
        servoturret = hardwareMap.get(Servo.class,"turret");

//        subsystem = new TemplateSubsystem(hardwareMap);
    }
    @Override
    public void init_loop() {//this is called repeatedly during initiallzation
//        follower.update();

        telemetry.update();
    }



    @Override
    public void start() {//this is called once when the driver presses start
        servoturret.setPosition(0.5);
    }

    @Override
    public void loop() {//this is called repeatedly during the op mode
//        follower.update();
//        driveWithPedro();
//        sendTelemetry();
        moveServoDpad();
        servoset();
        telemetry.addData("turret position",servoturret.getPosition());
        telemetry.addData("degrees",servovalueindegreees());
    }
    public void setservotodegree(double x){
        double y=-0.00236111*x+0.545;
        servoturret.setPosition(y);
    }
    public void moveServoDpad(){
        if (gamepad2.dpadUpWasPressed()){
            servoturret.setPosition(servoturret.getPosition()+0.01);
        }
        if(gamepad2.dpadDownWasPressed()){
            servoturret.setPosition(servoturret.getPosition()-0.01);
        }
    }
    public void servoset(){ //when cross is pressed it automatically sets the turret to 45,
        // when trinagle is pressed it sets it to 60
        if (gamepad2.crossWasPressed()){
            setservotodegree(45);
        }
        if (gamepad2.triangleWasPressed()){
            setservotodegree(60);
        }
    }
    public double servovalueindegreees(){
        double y =-423.52941* servoturret.getPosition() +230.82353;
        return y;
    }

}


