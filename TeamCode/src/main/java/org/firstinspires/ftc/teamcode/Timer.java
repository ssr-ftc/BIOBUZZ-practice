package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Holds multiple named, one-shot timers in one object.
 *
 * <pre>
 * Timer timer = new Timer();
 * timer.createNew("intakeDelay");
 * timer.start("intakeDelay");
 *
 * if (timer.checkSeconds("intakeDelay", 0.5)) {
 *     // Runs once, after the timer has been running for at least 0.5 seconds.
 * }
 *
 * if (timer.checkSecondsLast("intakeDelay", 1.0)) {
 *     // Runs once, then stops this timer.
 * }
 * </pre>
 */
public class Timer {
    private final Map<String, TimerState> timers = new HashMap<>();

    /** Creates or resets a stopped named timer. */
    public void createNew(String name) {
        validateName(name);
        timers.put(name, new TimerState());
    }

    /**
     * Starts a named timer from zero. Starting it again restarts the timer and re-arms its checks.
     */
    public void start(String name) {
        TimerState timer = getTimer(name);
        timer.startTimeNanos = System.nanoTime();
        timer.running = true;
        timer.firedChecks.clear();
        timer.firedLastChecks.clear();
    }

    /** Stops a named timer without removing it. */
    public void stop(String name) {
        getTimer(name).running = false;
    }

    /** Returns whether a named timer is currently running. */
    public boolean isRunning(String name) {
        return getTimer(name).running;
    }

    /**
     * Returns true once after {@code seconds} have elapsed. The timer stays running.
     * The same check is re-armed by the next {@link #start(String)} call.
     */
    public boolean checkSeconds(String name, double seconds) {
        validateSeconds(seconds);
        TimerState timer = getTimer(name);
        long checkKey = Double.doubleToLongBits(seconds);

        if (!timer.running || timer.firedChecks.contains(checkKey) || elapsedSeconds(timer) < seconds) {
            return false;
        }

        timer.firedChecks.add(checkKey);
        return true;
    }

    /**
     * Returns true once after {@code seconds} have elapsed, then stops that named timer.
     * This is an open-ended check: it still fires if the loop reaches it later than the threshold.
     */
    public boolean checkSecondsLast(String name, double seconds) {
        validateSeconds(seconds);
        TimerState timer = getTimer(name);
        long checkKey = Double.doubleToLongBits(seconds);

        if (!timer.running || timer.firedLastChecks.contains(checkKey) || elapsedSeconds(timer) < seconds) {
            return false;
        }

        timer.firedLastChecks.add(checkKey);
        timer.running = false;
        return true;
    }

    private TimerState getTimer(String name) {
        validateName(name);
        TimerState timer = timers.get(name);
        if (timer == null) {
            throw new IllegalArgumentException(
                    "No timer named '" + name + "'. Call createNew(\"" + name + "\") first.");
        }
        return timer;
    }

    private static double elapsedSeconds(TimerState timer) {
        return (System.nanoTime() - timer.startTimeNanos) / 1_000_000_000.0;
    }

    private static void validateName(String name) {
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("Timer name cannot be blank.");
        }
    }

    private static void validateSeconds(double seconds) {
        if (seconds < 0 || Double.isNaN(seconds) || Double.isInfinite(seconds)) {
            throw new IllegalArgumentException("Seconds must be a finite value greater than or equal to zero.");
        }
    }

    private static class TimerState {
        private long startTimeNanos;
        private boolean running;
        private final Set<Long> firedChecks = new HashSet<>();
        private final Set<Long> firedLastChecks = new HashSet<>();
    }
}
