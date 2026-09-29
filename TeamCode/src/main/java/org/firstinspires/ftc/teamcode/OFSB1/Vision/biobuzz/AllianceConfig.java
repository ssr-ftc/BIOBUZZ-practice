package org.firstinspires.ftc.teamcode.OFSB1.Vision.biobuzz;

/**
 * Single place the selected alliance lives. TeleOp reads it; vision never
 * prompts for it again after {@link #lock()}.
 */
public final class AllianceConfig {

    private static Alliance alliance;
    private static boolean locked;

    private AllianceConfig() {
    }

    /** Call from OpMode init() so a previous match cannot leak its color. */
    public static void reset() {
        alliance = null;
        locked = false;
    }

    /** Ignored after {@link #lock()}. Does not change the value if {@code selected} is null. */
    public static void select(Alliance selected) {
        if (!locked && selected != null) {
            alliance = selected;
        }
    }

    public static void lock() {
        locked = true;
    }

    public static boolean isLocked() {
        return locked;
    }

    /** Null until the driver picks a color during INIT. */
    public static Alliance get() {
        return alliance;
    }

    public static boolean isSelected() {
        return alliance != null;
    }
}
