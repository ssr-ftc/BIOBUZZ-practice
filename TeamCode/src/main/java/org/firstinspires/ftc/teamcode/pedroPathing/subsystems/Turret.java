package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Turret {
    private static final Pose BLUE_GARDEN_HIVE = new Pose(84.36, 81.4, 0);
    private static final Pose BLUE_OPPOSITE_HIVE = new Pose(59.64, 81.4, 0);
    private static final Pose RED_GARDEN_HIVE = new Pose(59.64, 62.6, 0);
    private static final Pose RED_OPPOSITE_HIVE = new Pose(84.36, 62.6, 0);
    public static double deg;
    final double scalerToDegrees = -0.838888;

    private DcMotorEx turret;
    public Servo cam;
    public Turret(HardwareMap hardwareMap) {
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
    public static double i = 0.0;
    public static double d = 0.0006;
    public static double kF = 0.000;
    private double targetPos = 0;

    public void setPositionTicks(double x) {
        targetPos = x;
    }

    public void setPositionDeg(double x) {
        while (x < -90) x += 360;
        while (x >= 270) x -= 360;
        targetPos = scalerToDegrees * x;
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

    public void resetTurretEncoders(){
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getPosition() {
        return turret.getCurrentPosition();
    }

    public double getPositionDeg() {
        return turret.getCurrentPosition() / scalerToDegrees;
    }

    public void turretAngleBlueGarden(Pose robot) {
         AutoAllignTo(BLUE_GARDEN_HIVE, robot);
    }

    public void turretAngleBlueOpposite(Pose robot) {
         AutoAllignTo(BLUE_OPPOSITE_HIVE, robot);
    }

    public void turretAngleRedGarden(Pose robot) {
         AutoAllignTo(RED_GARDEN_HIVE, robot);
    }

    public void turretAngleRedOpposite(Pose robot) {
         AutoAllignTo(RED_OPPOSITE_HIVE, robot);
    }

    public void AutoAllignTo(Pose point, Pose robot) {
        double headingDeg = Math.toDegrees(robot.heading());
        double fieldDeg = Math.toDegrees(Math.atan2(point.y() - robot.y(), point.x() - robot.x()));
        deg = -(fieldDeg - headingDeg);
        setPositionDeg(deg);
    }
}
