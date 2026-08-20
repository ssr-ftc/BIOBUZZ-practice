package org.firstinspires.ftc.teamcode.OFSB2.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OFSB2.Auto.Constants;

public class CustomFollower {

    public Follower pedro;
    private Telemetry telemetry;

    // --- MANUAL RAMPING VARIABLES ---
    private boolean isRamping = false;
    private double rampStartT, rampEndT, rampStartPower, rampEndPower;

    // --- AUTO-PHYSICS VARIABLES ---
    private boolean isAutoCalculating = false;
    private double mu = 0.5; // Lowered default to 0.5 to prevent mecanum drift
    private double robotMaxSpeed = 80.0;
    private final double gravity = 386.1; // in/s^2

    // TUNE THIS: How far ahead the robot looks to brake (0.05 = 5% ahead)
    // Increase to 0.08 or 0.1 if it brakes too late. Decrease to 0.03 if it brakes too early.
    public double lookaheadAmount = 0.05;

    public CustomFollower(HardwareMap hardwareMap, Telemetry telemetry) {
        pedro = Constants.createFollower(hardwareMap);
        this.telemetry = telemetry;
    }

    /**
     * MANUAL MODE: Call this to manually scale power between two T-values.
     */
    public void acceleration(double startT, double endT, double startPower, double endPower) {
        this.rampStartT = startT;
        this.rampEndT = endT;
        this.rampStartPower = startPower;
        this.rampEndPower = endPower;

        this.isRamping = true; // Turn the manual math engine ON
        this.isAutoCalculating = false; // Turn the physics engine OFF
    }

    /**
     * AUTO PHYSICS MODE: Call this to let the robot auto-calculate cornering speeds.
     * @param frictionCoefficient Usually 0.4 to 0.6 for mecanum wheels to prevent sliding.
     * @param maxSpeed Your robot's absolute top speed in inches per second.
     */
    public void autoAcceleration(double frictionCoefficient, double maxSpeed) {
        this.mu = frictionCoefficient;
        this.robotMaxSpeed = maxSpeed;

        this.isAutoCalculating = true; // Turn the physics engine ON
        this.isRamping = false; // Turn the manual math engine OFF
    }

    public void update() {
        if (pedro.isBusy()) {
            double currentT = pedro.getCurrentTValue();

            // 1A. RUN PHYSICS MATH ENGINE (Now with Lookahead!)
            if (isAutoCalculating) {

                // Calculate a future T-value to look ahead on the path
                // Math.min ensures we don't accidentally ask for a point past the end of the curve
                double lookaheadT = Math.min(1.0, currentT + lookaheadAmount);

                // Get the curvature of the path at that FUTURE point
                double curvature = pedro.getCurrentPath().getCurvature(lookaheadT);

                // Convert curvature to radius
                double radius = (Math.abs(curvature) < 0.00001) ? 10000.0 : Math.abs(1.0 / curvature);

                // The Physics Equation: v = sqrt(mu * g * R)
                double maxSafeSpeed = Math.sqrt(mu * gravity * radius);

                // Convert safe speed into a power percentage (0.0 to 1.0)
                double dynamicPowerLimit = maxSafeSpeed / robotMaxSpeed;

                // Clamp it so we never send more than 100% power to Pedro
                pedro.setMaxPower(Math.min(1.0, dynamicPowerLimit));

                // 1B. OR RUN MANUAL RAMPING ENGINE
            } else if (isRamping) {
                if (currentT >= rampStartT && currentT <= rampEndT) {
                    double progress = (currentT - rampStartT) / (rampEndT - rampStartT);
                    double smoothPower = rampStartPower + (progress * (rampEndPower - rampStartPower));
                    pedro.setMaxPower(smoothPower);

                } else if (currentT > rampEndT) {
                    pedro.setMaxPower(rampEndPower);
                    isRamping = false;
                }
            }
        }

        // 2. Run Pedro Pathing with whichever limits were just set
        pedro.update();
        telemetryForAcceleration();
    }

    void telemetryForAcceleration() {
        double liveT = pedro.isBusy() ? pedro.getCurrentTValue() : 0.0;
        double liveV = pedro.isBusy() ? pedro.getVelocity().getMagnitude() : 0.0;
        double liveA = pedro.isBusy() ? pedro.getAcceleration().getMagnitude() : 0.0;
        double liveP = pedro.isBusy() ? pedro.getMaxPowerScaling() : 0.0;

        if (telemetry != null) {
            telemetry.addData("Active Mode", isAutoCalculating ? "AUTO-PHYSICS" : (isRamping ? "MANUAL RAMPING" : "NONE"));
            telemetry.addData("Live T", liveT);
            telemetry.addData("Current Velocity", liveV);
            telemetry.addData("Current Set Power Scale", liveP);
        }
    }
}