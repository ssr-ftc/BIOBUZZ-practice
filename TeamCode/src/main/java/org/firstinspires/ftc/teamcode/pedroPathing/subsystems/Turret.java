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
   static double deg;
   static boolean isAuto = false;
   static double totalDeg;
   final double scalerToDegrees = 2.1;

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
    public static double p = 0.0078; //0.002
    public static double i = 0.055; //0
    public static double d = 0.00039; //0
    public static double kF = 0.0002;
    private double targetPos = 0;

    public void setPositionTicks(double x) {
        targetPos = x;
    }

    public void setPositionDeg(double x) {
        isAuto = false;
        if (x < -90) x += 360;
        targetPos = scalerToDegrees * x;
    }

    public void run_using_pid() {
        if (isAuto) {
            double aim = deg;
            if (aim < -90) aim += 360;
            targetPos = scalerToDegrees * aim;
        }
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
        return turret.getCurrentPosition() / scalerToDegrees;
    }

    // blue hive cell in field coordinates (my estimate, edit these two if you know better)

    // type the robot's field x, y and heading (degrees), get the turret angle (degrees)
   /* public static double turretAngle(double robotX, double robotY, double robotHeadingDeg) {
        return turretAngleBlueGarden(robotX, robotY, robotHeadingDeg);
    }
    */

    public static double turretAngleBlueGarden(Pose robot) {
        return turretAngleTo(BLUE_GARDEN_HIVE, robot);
    }

    public static double turretAngleBlueOpposite(Pose robot) {
        return turretAngleTo(BLUE_OPPOSITE_HIVE, robot);
    }

    public static double turretAngleRedGarden(Pose robot) {
        return turretAngleTo(RED_GARDEN_HIVE, robot);
    }

    public static double turretAngleRedOpposite(Pose robot) {
        return turretAngleTo(RED_OPPOSITE_HIVE, robot);
    }

    private static double turretAngleTo(Pose hive, Pose robot) {
        double headingDeg = Math.toDegrees(robot.heading());
        double fieldDeg = Math.toDegrees(Math.atan2(hive.y() - robot.y(), hive.x() - robot.x()));
        deg = -(fieldDeg - headingDeg);
        while (deg < -90) deg += 360;
        while (deg >= 270) deg -= 360;
        isAuto = true;
        return deg;
    }
}
