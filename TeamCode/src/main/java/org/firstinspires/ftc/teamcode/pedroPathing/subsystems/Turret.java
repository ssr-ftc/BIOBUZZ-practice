package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Turret {
    public static  Pose GARDEN_HIVE = new Pose(59, 64, 0);
    public static  Pose FLOWER_HIVE = new Pose(59, 89, 0);
    public static final Pose GARDEN_HIVE_dist = new Pose(59, 55, 0);
    public static final  Pose FLOWER_HIVE_dist = new Pose(59, 89, 0);
    public static double face_coeff = 0;
    public static double deg;
    final double scalerToDegrees = -23.13888;

    private DcMotorEx encoder;
    CRServo turret;
    public Servo cam;
    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(CRServo.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = hardwareMap.get(DcMotorEx.class, "intake2");

        pid = new PIDFController(p, i, d, kF);
    }
    public void start(){
        pid.setPIDF(p, i, d, kF);
    }
    PIDFController pid;
    public double power;
    public static double p = 0.0002;
    public static double i = 0;
    public static double d = 0.000013;
    public static double kF = 0;
    public static double kS = 0.07;
    public static double Deadband = 70;
    private double targetPos = 0;

    public void setPositionTicks(double x) {
        targetPos = x;
    }

    public void setPositionDeg(double x) {
        while (x < -370) x += 360;
        while (x >= 17) x -= 360;
        targetPos = scalerToDegrees * x;
    }

    public void run_using_pid() {
        double currentPos = encoder.getCurrentPosition();
        double error = targetPos - currentPos;
        double pidOutput = pid.calculate(currentPos, targetPos);

        // Only apply static friction if we actually have an error,
        // otherwise it will chatter/oscillate around the setpoint.
        double staticFriction = (Math.abs(error) > Deadband) ? Math.signum(error) * kS : 0;

        power = pidOutput + staticFriction;
        power = Math.max(-1.0, Math.min(1.0, power));
        turret.setPower(power);
    }
    public void turretOff(){
        turret.setPower(0);
    }

    public void holdCurrentPosition() {
        targetPos = encoder.getCurrentPosition();
    }

    public void resetTurretEncoders(){
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getPosition() {
        return encoder.getCurrentPosition();
    }

    public double getTargetDeg() {
        return targetPos / scalerToDegrees;
    }

    public double getPositionDeg() {
        return encoder.getCurrentPosition() / scalerToDegrees;
    }


    public void faceGardenHive(Pose robot) {
         AutoAllignTo(GARDEN_HIVE, robot);
    }
    public void faceFlowerHive(Pose robot) {
         AutoAllignTo(FLOWER_HIVE, robot);
    }
    public double distanceToGardenHive(Pose robot) {
         return distanceTo(GARDEN_HIVE_dist, robot);
    }
    public double distanceToFlowerHive(Pose robot) {
         return distanceTo(FLOWER_HIVE_dist, robot);
    }

    public void AutoAllignTo(Pose point, Pose robot) {
        double headingDeg = Math.toDegrees(robot.heading());
        double fieldDeg = Math.toDegrees(Math.atan2(point.y() - robot.y(), point.x() - robot.x()));
        double xDiff = (point.x()-robot.x())*face_coeff;
        deg = -(fieldDeg - headingDeg);
        if(robot.y()>71)setPositionDeg(deg-xDiff);
        else setPositionDeg(deg+xDiff);
    }
    public double distanceTo(Pose point, Pose robot){
        double x = robot.x()-point.x();
        double y = robot.y()-point.y();
        double dist = Math.hypot(x,y);
        return dist;
    }
}
