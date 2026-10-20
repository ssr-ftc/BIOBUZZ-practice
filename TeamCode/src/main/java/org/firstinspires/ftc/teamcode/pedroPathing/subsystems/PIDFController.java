package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

public class PIDFController {
    public double p, i, d, kF;
    private double lastError = 0;
    private double integralSum = 0;
    private static final double MAX_INTEGRAL = 1000; // tune; limits windup

    public PIDFController(double p, double i, double d, double kF) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.kF = kF;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
    }

    public double calculate(double current, double target) {
        double error = target - current;
        integralSum += error;
        integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));
        double derivative = error - lastError;
        lastError = error;
        return (p * error) + (i * integralSum) + (d * derivative) + (kF * target);
    }
}
