package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

//the front intake is the passive one
    //the back intake is the one that spits
    private DcMotor intakeMotor;
    private Servo flower;
    private Servo back;
    private Servo front;

    private double flowerUpPos = 0.99;
    private double flowerDownPos = 0.13;
    private double frontUpPos = 0.99;
    private double frontDownPos = 0.13;
    private double backUpPos = 0.99;
    private double backDownPos = 0.13;
    private static final double intake_power = -1;
    private static final double outtake_power = 1;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flower = hardwareMap.get(Servo.class, "flower_intake");
        front = hardwareMap.get(Servo.class, "front_intake");
        back = hardwareMap.get(Servo.class, "back_intake");
        front_up();
        back_up();
    }
    public void start(){
        intake_off();
        flower_up();
    }

    public void flower_up(){
        flower.setPosition(flowerUpPos);
    }
    public void flower_down(){flower.setPosition(flowerDownPos);}
    public void front_up(){front.setPosition(frontUpPos);}
    public void front_down(){front.setPosition(frontDownPos);}
    public void back_up(){back.setPosition(backUpPos);}
    public void back_down(){back.setPosition(backDownPos);}
    public boolean isFlowerDown(){
        return flower.getPosition() < flowerUpPos -0.1;
    }
    public void intake_on() {
        intakeMotor.setPower(intake_power);
    }

    public void intake_off() {
        intakeMotor.setPower(0);
    }

    public void outtake() {
        intakeMotor.setPower(outtake_power);
    }

    public boolean intaking() {
        return intakeMotor.getPower() != 0;
    }
    public boolean outtaking() {
        return intakeMotor.getPower() > 0;
    }

    public double getMotorPower() {
        return intakeMotor.getPower();
    }
}
//hi