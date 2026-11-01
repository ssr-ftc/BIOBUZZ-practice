package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Robot0;

public class CustomFollower {

    public Follower pedro;
    private Telemetry telemetry;
    private boolean isAuto; // True when used from an autonomous OpMode, false for TeleOp

    // --- MANUAL RAMPING VARIABLES ---
    private boolean isRamping = false;
    private double rampStartT, rampEndT, rampStartPower, rampEndPower;
    private boolean rampLocked = false; // True once the ramp has started on a running path

    // --- AUTO-PHYSICS VARIABLES ---
    private boolean isAutoCalculating = false;
    private double mu = 0.5; // Lowered default to 0.5 to prevent mecanum drift
    private double robotMaxSpeed = 80.0;
    private final double gravity = 386.1; // in/s^2

    // TUNE THIS: How far ahead the robot looks to brake (0.05 = 5% ahead)
    // Increase to 0.08 or 0.1 if it brakes too late. Decrease to 0.03 if it brakes too early.
    public double lookaheadAmount = 0.05;

    public CustomFollower(HardwareMap hardwareMap, Telemetry telemetry) {
        this(telemetry, Robot0.createFollower(hardwareMap));
    }

    public CustomFollower(HardwareMap hardwareMap, Telemetry telemetry, Follower follower) {
        this(telemetry, follower);
    }

    private CustomFollower(Telemetry telemetry, Follower follower) {
        pedro = follower;
        this.telemetry = telemetry;
        this.isAuto = isAuto;
    }

    /**
     * MANUAL MODE: Call this to manually scale power between two T-values.
     */
    public void acceleration(double startT, double endT, double startPower, double endPower) {
        this.rampStartT = startT;
        this.rampEndT = endT;
        this.rampStartPower = startPower;
        this.rampEndPower = endPower;
        this.rampLocked = false; // Unlock so the ramp locks onto the next path

        this.isRamping = true; // Turn the manual math engine ON
        this.isAutoCalculating = false; // Turn the physics engine OFF
    }

    /**
     * AUTO PHYSICS MODE: Call this to let the robot auto-calculate cornering speeds.
     * @param frictionCoefficient Usually 0.4 to 0.6 for mecanum wheels to prevent sliding.
     * @param maxSpeed Your robot's absolute top speed in inches per second.
     */
    public void enableAutoPhysics(double frictionCoefficient, double maxSpeed) {
        this.mu = frictionCoefficient;
        this.robotMaxSpeed = maxSpeed;

        this.isAutoCalculating = true; // Turn the physics engine ON
        this.isRamping = false; // Turn the manual math engine OFF
    }

    public void update() {
        if (pedro.isBusy()) {
            double currentT = pedro.parametricCompletion();

            // 1A. RUN PHYSICS MATH ENGINE (Now with Lookahead!)
            if (isAutoCalculating) {

                // Calculate a future T-value to look ahead on the path
                double lookaheadT = Math.min(1.0, currentT + lookaheadAmount);
                double curvature = pedro.curvatureAt(lookaheadT);
                double radius = (Math.abs(curvature) < 0.00001) ? 10000.0 : Math.abs(1.0 / curvature);
                double maxSafeSpeed = Math.sqrt(mu * gravity * radius);
                double dynamicPowerLimit = maxSafeSpeed / robotMaxSpeed;

                    setMaxPower(Math.min(1.0, dynamicPowerLimit));

                // 1B. OR RUN MANUAL RAMPING ENGINE
            } else if (isRamping) {
                // Mark the ramp as started now that a path is actually running
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
        } else {
            // Path finished: Reset ramping state & restore 100% max power immediately
            if (isRamping) {
                endRamp();
            }
        }

        // 2. Run Pedro Pathing with whichever limits were just set
        pedro.update();
        telemetryForAcceleration();
    }

    void telemetryForAcceleration() {
        double liveT = pedro.isBusy() ? pedro.parametricCompletion() : 0.0;
        double liveV = pedro.isBusy() ? pedro.velocity().linSpeed() : 0.0;
        double liveP = pedro.isBusy() ? currentMaxPower() : 0.0;

        if (telemetry != null) {
            telemetry.addData("Active Mode", isAutoCalculating ? "AUTO-PHYSICS" : (isRamping ? "MANUAL RAMPING" : "NONE"));
            telemetry.addData("Live T", liveT);
            telemetry.addData("Current Velocity", liveV);
            telemetry.addData("Current Set Power Scale", liveP);
        }
    }

    // --- RAMP HELPERS ---
    private void endRamp() {
        setMaxPower(1.0);
        isRamping = false;
        rampLocked = false;
    }

    private void prepareToFollow() {
        // A ramp that already locked onto a path belongs to a path that has
        // finished (or is being replaced), so end it.
        // A ramp set by acceleration() just before this call hasn't locked yet
        // (rampLocked is false), so it is kept for the path about to start.
        if (isRamping && rampLocked) {
            endRamp();
        }
        setMaxPower(1.0);
    }

    // --- DELEGATED PEDRO FOLLOWER METHODS ---
    public boolean isBusy() {
        return pedro.isBusy();
    }

    public void followPath(Path path, boolean holdEnd) {
        prepareToFollow();
        pedro.holdEnd.set(holdEnd);
        pedro.follow(path);
    }

    public void followPath(Path path, double maxPower, boolean holdEnd) {
        followPath(path, holdEnd);
        setMaxPower(maxPower);
    }

    public void followPath(Path path) {
        followPath(path, false);
    }

    public void setPose(Pose pose) {
        pedro.setPose(pose);
    }

    public Pose getPose() {
        return pedro.pose();
    }

    public void setMaxPower(double power) {
        if (pedro.algorithm() instanceof Foresight) {
            ((Foresight) pedro.algorithm()).config.maxPathSpeed.set(power);
        }
    }

    private double currentMaxPower() {
        if (pedro.algorithm() instanceof Foresight) {
            return ((Foresight) pedro.algorithm()).config.maxPathSpeed.get();
        }
        return 1;
    }

    public double getCurrentTValue() {
        return pedro.parametricCompletion();
    }
}