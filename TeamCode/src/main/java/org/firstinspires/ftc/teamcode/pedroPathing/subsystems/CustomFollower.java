package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot0;

public class CustomFollower {

    public Follower pedro;
    private Telemetry telemetry;

    private boolean isRamping = false;
    private double rampStartT, rampEndT, rampStartPower, rampEndPower;
    private boolean rampLocked = false;

    private boolean isAutoCalculating = false;
    private double mu = 0.5;
    private double robotMaxSpeed = 80.0;
    private final double gravity = 386.1;

    public double lookaheadAmount = 0.05;

    public CustomFollower(HardwareMap hardwareMap, Telemetry telemetry) {
        pedro = Robot0.createFollower(hardwareMap);
        this.telemetry = telemetry;
    }

    public void acceleration(double startT, double endT, double startPower, double endPower) {
        this.rampStartT = startT;
        this.rampEndT = endT;
        this.rampStartPower = startPower;
        this.rampEndPower = endPower;
        this.rampLocked = false;

        this.isRamping = true;
        this.isAutoCalculating = false;
    }

    public void enableAutoPhysics(double frictionCoefficient, double maxSpeed) {
        this.mu = frictionCoefficient;
        this.robotMaxSpeed = maxSpeed;

        this.isAutoCalculating = true;
        this.isRamping = false;
    }

    public void update() {
        if (pedro.isBusy()) {
            double currentT = pedro.parametricCompletion();

            if (isAutoCalculating) {
                double lookaheadT = Math.min(1.0, currentT + lookaheadAmount);
                double curvature = pedro.curvatureAt(lookaheadT);
                double radius = (Math.abs(curvature) < 0.00001) ? 10000.0 : Math.abs(1.0 / curvature);
                double maxSafeSpeed = Math.sqrt(mu * gravity * radius);
                double dynamicPowerLimit = maxSafeSpeed / robotMaxSpeed;

                setMaxPower(Math.min(1.0, dynamicPowerLimit));
            } else if (isRamping) {
                rampLocked = true;

                if (currentT < rampStartT) {
                    setMaxPower(1.0);
                } else if (currentT <= rampEndT) {
                    double progress = (currentT - rampStartT) / Math.max(1e-6, rampEndT - rampStartT);
                    double smoothPower = rampStartPower + (progress * (rampEndPower - rampStartPower));
                    setMaxPower(smoothPower);
                } else {
                    endRamp();
                }
            }
        } else if (isRamping) {
            endRamp();
        }

        pedro.update();
        telemetryForAcceleration();
    }

    void telemetryForAcceleration() {
        double liveT = pedro.isBusy() ? pedro.parametricCompletion() : 0.0;
        double liveV = pedro.isBusy() ? pedro.velocity().linSpeed() : 0.0;

        if (telemetry != null) {
            telemetry.addData("Active Mode", isAutoCalculating ? "AUTO-PHYSICS" : (isRamping ? "MANUAL RAMPING" : "NONE"));
            telemetry.addData("Live T", liveT);
            telemetry.addData("Current Velocity", liveV);
        }
    }

    private void endRamp() {
        setMaxPower(1.0);
        isRamping = false;
        rampLocked = false;
    }

    private void prepareToFollow() {
        if (isRamping && rampLocked) {
            endRamp();
        }
        setMaxPower(1.0);
    }

    public boolean isBusy() {
        return pedro.isBusy();
    }

    public void followPath(Path path, boolean holdEnd) {
        prepareToFollow();
        pedro.holdEnd.set(holdEnd);
        pedro.follow(path);
    }

    public void followPath(Path path) {
        followPath(path, true);
    }

    public void follow(Path path) {
        followPath(path, true);
    }

    public void setPose(Pose pose) {
        pedro.setPose(pose);
    }

    public Pose getPose() {
        return pedro.pose();
    }

    public Pose pose() {
        return pedro.pose();
    }

    public void setMaxPower(double power) {
        Robot0.setPathPower(power);
    }

    public double getCurrentTValue() {
        return pedro.parametricCompletion();
    }
}
