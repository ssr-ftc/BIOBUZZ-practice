package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class intakeLM0 {


    private DcMotor intakeMotor;
    public Servo flowerintake;
    public double floweruppos = 0.45; // not a real variable yet
    public double flowerdownpos = 0.45; // not a real variable yet
    private static final double intake_power = -1;
    private static final double outtake_power = 1;

    public intakeLM0(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class,"intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flowerintake = hardwareMap.get(Servo.class, "flower_intake");
    }
    public void start(){
        turn_off_intake();
//        flower_up();
    }

    public void flower_up(){
        flowerintake.setPosition(floweruppos);
    }

    public void flower_down(){
        flowerintake.setPosition(flowerdownpos);
    }
    public void turn_on_intake() {
        intakeMotor.setPower(intake_power);
    }

    public void turn_off_intake() {
        intakeMotor.setPower(0);
    }

    public void outtake() {
        intakeMotor.setPower(outtake_power);
    }

    public boolean isIntakeOn() {
        return intakeMotor.getPower() != 0;
    }

    public double getMotorPower() {
        return intakeMotor.getPower();
    }
}
