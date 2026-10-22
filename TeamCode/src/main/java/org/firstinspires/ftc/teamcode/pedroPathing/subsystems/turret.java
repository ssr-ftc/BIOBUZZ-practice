package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//@Config
public class turret {
    private static final double HIVE_X = 84.36;
    private static final double HIVE_Y = 81.4;
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
    public static double p = 0.0027;
    public static double i = 0.05;
    public static double d = 0.000001;
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
        power = pid.calculate(turret.getCurrentPosition(), targetPos);
        power = Math.max(-1, Math.min(1, power));
        turret.setPower(power);
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

    // blue hive cell in field coordinates (my estimate, edit these two if you know better)

    // type the robot's field x, y and heading (degrees), get the turret angle (degrees)
     public static double turretAngle(double robotX, double robotY, double robotHeadingDeg) {
        double fieldDeg = Math.toDegrees(Math.atan2(HIVE_Y - robotY, HIVE_X - robotX));
        deg = -(fieldDeg - robotHeadingDeg);
        while (deg < -90) deg += 360;
        while (deg >= 270) deg -= 360;

        return deg;
    }
}
//hi