package org.firstinspires.ftc.teamcode.OFSWB.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class intakelm2 {


    private final DcMotor intakeMotor;
    private final Servo armServo;

    private static final String MOTOR_NAME = "intake";
    private static final String SERVO_NAME = "lift_intake"; // <-- change to match your config

    private static final double arm_up = 0.29;
    private static final double arm_down = 0.13;
    private static final double ballheight = 0.22;

    // Position past which the motor should automatically turn on.
    // If your arm goes UP as position increases, flip the comparison in update().

    private static final double intake_power = 1.0;
    private static final double outtake_power = -1.0;



    public intakelm2(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armServo = hardwareMap.get(Servo.class, SERVO_NAME);
        armServo.setDirection(Servo.Direction.REVERSE);
    }

    /** Call every loop. Handles auto-turning the motor on/off based on arm position. */
    public void update() {

    }
    public void start() {
        down();
    }
    public void up_1(){
        armServo.setPosition(armServo.getPosition()+0.01);
    }

    public void down_1(){
        armServo.setPosition(armServo.getPosition()-0.01);
    }
    // ---------- ARM CONTROL ----------

    public void up() {
        armServo.setPosition(arm_up);
    }

    public void down() {
        armServo.setPosition(arm_down);
    }
    public void intake_ball_height(){
        armServo.setPosition(ballheight);
    }

    public void setArmPosition(double position) {
        armServo.setPosition(position);
    }

    public double getArmPosition() {
        return armServo.getPosition();
    }

    public double getMotorPower() {
        return intakeMotor.getPower();
    }
    public void intake_on (){
        intakeMotor.setPower(1);
    }
}
