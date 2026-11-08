package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {

//the front intake is the passive one
    //the back intake is the one that spits
    private DcMotorEx intakeMotor;
    private DcMotorEx intakeMotor2;
    private Servo flower;
    private Servo back;
    private Servo front;
    Rev2mDistanceSensor front_bottom;
    Rev2mDistanceSensor back_bottom;

    private double flowerUpPos = 0.89;
    private double flowerDownPos = 0.0894;

    private double frontUpPos = 0.38;
    private double frontDownPos = 0.5;

    private double backUpPos = 0.42;
    private double backDownPos = 0.54;
    private static final double intake_power = -1;
    private static final double outtake_power = 1;
    boolean backState,frontState;
    double totalBalls;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake2");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flower = hardwareMap.get(Servo.class, "flower");
        front = hardwareMap.get(Servo.class, "front");
        back = hardwareMap.get(Servo.class, "back");
        front_bottom = hardwareMap.get(Rev2mDistanceSensor.class, "front_bottom");
        back_bottom = hardwareMap.get(Rev2mDistanceSensor.class, "back_bottom");
        front_up();
        back_up();
    }

    public void start(){
        intake_off();
        front_up();
        back_up();
        flower_up();
    }

    public void flower_up(){
        flower.setPosition(flowerUpPos);
    }
    public void flower_down() {flower.setPosition(flowerDownPos);}
    public void front_up(){front.setPosition(frontUpPos);}
    public void front_down(){front.setPosition(frontDownPos);}
    public void back_up(){back.setPosition(backUpPos);}
    public void back_down(){back.setPosition(backDownPos);}
    public boolean isFlowerDown(){
        return flower.getPosition() < flowerUpPos -0.1;
    }
    public void intake_on() {
        intakeMotor.setPower(intake_power);
        intakeMotor2.setPower(intake_power);
    }

    public void intake_off() {
        intakeMotor.setPower(0);
        intakeMotor2.setPower(0);
    }

    public void outtake() {
        intakeMotor.setPower(outtake_power);
        intakeMotor2.setPower(outtake_power);
    }
    public double getDistanceFront_MM(){
        return back_bottom.getDistance(DistanceUnit.MM);
    }
    public double getDistanceBack_MM(){
        return front_bottom.getDistance(DistanceUnit.MM);
    }

    public boolean intaking() {
        return intakeMotor.getPower() < 0;
    }
    public boolean outtaking() {
        return intakeMotor.getPower() > 0;
    }

    public double getMotorPower() {
        return intakeMotor.getPower();
    }
    public boolean isBackUp(){
        return back.getPosition()<backDownPos-0.03;
    }
    public boolean isFrontUp(){
        return back.getPosition()<frontDownPos-0.03;
    }
    public void countBalls(){
        countBack();
        countFront();
    }

    public void countBack(){
        if(getDistanceBack_MM()<65 && !backState) backState=true;
        if(backState){
            if(getDistanceBack_MM()>80){
                totalBalls+=1;
                backState=false;
            }
        }
    }

    public void countFront(){
        if(getDistanceFront_MM()<65 && !frontState) frontState=true;
        if(frontState){
            if(getDistanceFront_MM()>80){
                totalBalls+=1;
                frontState=false;
            }
        }
    }
    public void resetBalls(){
        totalBalls = 0;
    }
    public double getBalls(){
        return totalBalls;
    }
}
//hi