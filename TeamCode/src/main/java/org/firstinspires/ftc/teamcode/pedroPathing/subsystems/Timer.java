package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * How to use:
 * 1. Import into opmode
 * 2. make a timers object like this
 *
 * private Timer timers;
 *
 * this object will hold all created timers, so you can have multiple events running parallel without the need to make multiple objects
 * 3. Initialize the timers object inside init() method
 *
 * timers = new Timer();
 *
 * 4. Create as many timers as needed indide start() method like this
 *
 * timers.createNew("intake");
 * timers.createNew("depo");
 *
 * 5. inside loop() method start your timer using
 * timers.start("intake");
 * (do this when a button was pressed for example)
 *
 * 6. inside loop() method check for a specific time since the timer started like this
 *
 * if(timers.checkSeconds("intake",0.5))
 * {
 *      add stuff here to do when time reaches 0.5 seconds since start
 * }
 * if(timers.checkSeconds("intake",1))
 * {
 *      add stuff here to do when time reaches 1 seconds since start
 * }
 * note that things will only run one time once the timer reaches the time
 *
 * 7. if you want for the check to be last use
 * timers.checkSecondsLast("intake",1);
 * this ensures that timer turns off after it reaches 1 second
 *
 * 8. if you want to turn off a timer without checking for time use
 * timers.stop("intake");
 */
public class Timer {
    private final Map<String, ElapsedTime> timers = new HashMap<>();
    private final Set<String> runningTimers = new HashSet<>();
    private final Map<String, Set<Long>> firedChecks = new HashMap<>();
    private final Map<String, Set<Long>> firedLastChecks = new HashMap<>();

    /** Creates or resets a stopped named timer. */
    public void createNew(String name) {
        validateName(name);
        timers.put(name, new ElapsedTime());
        runningTimers.remove(name);
        firedChecks.put(name, new HashSet<>());
        firedLastChecks.put(name, new HashSet<>());
    }

    /**
     * Starts a named timer from zero. Starting it again restarts the timer and re-arms its checks.
     */
    public void start(String name) {
        getTimer(name).reset();
        runningTimers.add(name);
        firedChecks.get(name).clear();
        firedLastChecks.get(name).clear();
    }

    /** Stops a named timer without removing it. */
    public void stop(String name) {
        getTimer(name);
        runningTimers.remove(name);
    }

    /** Returns whether a named timer is currently running. */
    public boolean isRunning(String name) {
        getTimer(name);
        return runningTimers.contains(name);
    }

    /**
     * Returns true once after {@code seconds} have elapsed. The timer stays running.
     * The same check is re-armed by the next {@link #start(String)} call.
     */
    public boolean checkSeconds(String name, double seconds) {
        validateSeconds(seconds);
        ElapsedTime timer = getTimer(name);
        long checkKey = Double.doubleToLongBits(seconds);

        if (!runningTimers.contains(name) || firedChecks.get(name).contains(checkKey) || timer.seconds() < seconds) {
            return false;
        }

        firedChecks.get(name).add(checkKey);
        return true;
    }

    /**
     * Returns true once after {@code seconds} have elapsed, then stops that named timer.
     * This is an open-ended check: it still fires if the loop reaches it later than the threshold.
     */
    public boolean checkSecondsLast(String name, double seconds) {
        validateSeconds(seconds);
        ElapsedTime timer = getTimer(name);
        long checkKey = Double.doubleToLongBits(seconds);

        if (!runningTimers.contains(name) || firedLastChecks.get(name).contains(checkKey) || timer.seconds() < seconds) {
            return false;
        }

        firedLastChecks.get(name).add(checkKey);
        stop(name);
        return true;
    }

    private ElapsedTime getTimer(String name) {
        validateName(name);
        ElapsedTime timer = timers.get(name);
        if (timer == null) {
            throw new IllegalArgumentException(
                    "No timer named '" + name + "'. Call createNew(\"" + name + "\") first.");
        }
        return timer;
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
}
