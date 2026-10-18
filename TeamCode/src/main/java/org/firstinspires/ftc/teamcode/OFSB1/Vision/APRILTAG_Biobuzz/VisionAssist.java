package org.firstinspires.ftc.teamcode.OFSB1.Vision.APRILTAG_Biobuzz;

/**
 * Optional heading assist. Translation is never touched.
 * A stick deflection past {@link BioBuzzVisionConfig#DRIVER_TURN_OVERRIDE}
 * returns the driver's turn command for that loop.
 */
public final class VisionAssist {

    private AssistMode mode = AssistMode.MANUAL;

    public AssistMode getMode() {
        return mode;
    }

    public void setEnabled(boolean enabled) {
        if (!enabled) {
            mode = AssistMode.MANUAL;
        } else if (mode == AssistMode.MANUAL) {
            mode = AssistMode.VISION_ASSIST;
        }
    }

    public boolean isEnabled() {
        return mode != AssistMode.MANUAL;
    }

    /**
     * @param driverTurnStick right stick X, FTC sign (right is positive)
     * @param driverTurnCommand the turn already computed for manual drive
     * @return turn power to pass to the drivetrain
     */
    public double turnPower(VisionState state, double driverTurnStick, double driverTurnCommand) {
        if (mode == AssistMode.MANUAL) {
            return driverTurnCommand;
        }
        if (Math.abs(driverTurnStick) >= BioBuzzVisionConfig.DRIVER_TURN_OVERRIDE) {
            mode = AssistMode.VISION_ASSIST;
            return driverTurnCommand;
        }
        if (state == null || !state.hasValidTarget()) {
            mode = AssistMode.VISION_ASSIST;
            return driverTurnCommand;
        }

        double bearing = state.getTargetBearing();
        if (!Double.isFinite(bearing)) {
            mode = AssistMode.VISION_ASSIST;
            return driverTurnCommand;
        }

        if (state.isTargetShootable() && Math.abs(bearing) <= BioBuzzVisionConfig.READY_BEARING_DEG) {
            mode = AssistMode.READY_TO_SHOOT;
        } else {
            mode = AssistMode.AIMING;
        }

        // Positive bearing: target is left of forward, so turn counterclockwise.
        double turn = BioBuzzVisionConfig.AIM_KP * Math.toRadians(bearing);
        double max = BioBuzzVisionConfig.AIM_MAX_TURN;
        if (turn > max) {
            return max;
        }
        if (turn < -max) {
            return -max;
        }
        return turn;
    }
}
