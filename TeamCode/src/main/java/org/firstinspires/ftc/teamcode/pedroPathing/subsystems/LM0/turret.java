package org.firstinspires.ftc.teamcode.pedroPathing.subsystems.LM0;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class turret {
    private static final double BLUE_GARDEN_HIVE_X = 84.36;
    private static final double BLUE_GARDEN_HIVE_Y = 81.4;
    private static final double BLUE_OPPOSITE_HIVE_X = 59.64;
    private static final double BLUE_OPPOSITE_HIVE_Y = 81.4;
    private static final double RED_GARDEN_HIVE_X = 59.64;
    private static final double RED_GARDEN_HIVE_Y = 62.6;
    private static final double RED_OPPOSITE_HIVE_X = 84.36;
    private static final double RED_OPPOSITE_HIVE_Y = 62.6;
   static double deg;
   static double totalDeg;


    private DcMotorEx turret;
    public Servo cam;
    public turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDFController(p, i, d, kF);
    }
    public void start(){
        pid.setPIDF(p, i, d, kF);
    }
    PIDFController pid;
    public double power;
    public static double p = 0.015;
    public static double i = 0.05;
    public static double d = 0.0005;
    public static double kF = 0;
    private double targetPos = 0;


    public void setPositionTicks(double x) {
        targetPos = x;
    }
    public void setPositionDeg(double x) {
        if(x < -90) x += 360;
        targetPos = 3.47778 * x;
    }

    public void run_using_pid() {
        pid.setPIDF(p, i, d, kF);
        power = pid.calculate(turret.getCurrentPosition(), targetPos);
        power = Math.max(-1, Math.min(1, power));
        turret.setPower(power);
    }
    public void holdCurrentPosition() {
        targetPos = turret.getCurrentPosition();
    }

    public void ResetTurretEncoders(){
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public double getPosition() {
        return turret.getCurrentPosition();
    }

    public double getPositionDeg() {
        return turret.getCurrentPosition() / 3.47778;
    }

    public static double turretAngle(double robotX, double robotY, double robotHeadingDeg) {
        return turretAngleBlueGarden(robotX, robotY, robotHeadingDeg);
    }

    public static double turretAngleBlueGarden(double robotX, double robotY, double robotHeadingDeg) {
        return turretAngleTo(BLUE_GARDEN_HIVE_X, BLUE_GARDEN_HIVE_Y, robotX, robotY, robotHeadingDeg);
    }

    public static double turretAngleBlueOpposite(double robotX, double robotY, double robotHeadingDeg) {
        return turretAngleTo(BLUE_OPPOSITE_HIVE_X, BLUE_OPPOSITE_HIVE_Y, robotX, robotY, robotHeadingDeg);
    }

    public static double turretAngleRedGarden(double robotX, double robotY, double robotHeadingDeg) {
        return turretAngleTo(RED_GARDEN_HIVE_X, RED_GARDEN_HIVE_Y, robotX, robotY, robotHeadingDeg);
    }

    public static double turretAngleRedOpposite(double robotX, double robotY, double robotHeadingDeg) {
        return turretAngleTo(RED_OPPOSITE_HIVE_X, RED_OPPOSITE_HIVE_Y, robotX, robotY, robotHeadingDeg);
    }

    private static double turretAngleTo(double hiveX, double hiveY, double robotX, double robotY, double robotHeadingDeg) {
        double fieldDeg = Math.toDegrees(Math.atan2(hiveY - robotY, hiveX - robotX));
        deg = -(fieldDeg - robotHeadingDeg);
        while (deg < -90) deg += 360;
        while (deg >= 270) deg -= 360;
        return deg;
    }
}
//hi