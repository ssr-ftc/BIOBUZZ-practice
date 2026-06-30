package org.firstinspires.ftc.teamcode.pedroPathing.templates;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Template subsystem for grouping hardware and robot-specific helper functions.
 *
 * Rename the hardware strings to match your Driver Station robot configuration.
 */
public class TemplateSubsystem {

    private static final double OPEN_POSITION = 0.25;//these are constants that are to be manually tuned
    private static final double CLOSED_POSITION = 0.75;//these are constants that are to be manually tuned

    //below is how you declare hardware objects
    public DcMotorEx testMotor;
    public Servo testServo;
    public CRServo crServo;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;

    public TemplateSubsystem(HardwareMap hardwareMap) {//this is a constructor that will be called in init of the teleop file
        testMotor = hardwareMap.get(DcMotorEx.class, "name");//these lines are how the code knows what motor to use
        testServo = hardwareMap.get(Servo.class, "name");//the name should be replaced with whatever you have in the robot configuration
        crServo = hardwareMap.get(CRServo.class, "name");
        colorSensor = hardwareMap.get(ColorSensor.class, "name");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "name");

        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //below are examples of functions that can be used in the teleop file

    public void setMotorPower(double power) {
        testMotor.setPower(power);
    }

    public int getMotorPosition() {
        return testMotor.getCurrentPosition();
    }
    public double getMotorVelocity() {
        return testMotor.getVelocity();
    }

    public void openServo() {
        testServo.setPosition(OPEN_POSITION);
    }

    public void closeServo() {
        testServo.setPosition(CLOSED_POSITION);
    }

    public double getServoPosition() {
        return testServo.getPosition();
    }

    public double getDistanceInches() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public boolean seesRed() {
        return colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green();
    }

    public boolean objectIsClose(double inches) {
        return getDistanceInches() <= inches;
    }

    public void stopAll() {
        testMotor.setPower(0.0);
        crServo.setPower(0.0);
    }
}
