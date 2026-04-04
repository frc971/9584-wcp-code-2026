package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/**
 * Runtime-toggleable experiment flags for diagnosing loop overruns.
 *
 * Two modes:
 *   MANUAL  – flags are controlled via SmartDashboard booleans (default).
 *   AUTO    – an automated test sequence runs each experiment for a fixed
 *             duration, with a baseline phase between each. Enable by setting
 *             Experiments/AutoRun = true in SmartDashboard before enabling.
 *
 * Default: all optimizations OFF (baseline behavior unchanged).
 */
public class LoopExperiments {
    private static final String PREFIX = "Experiments/";

    // === Experiment flags ===
    public static boolean skipTempReads = false;
    public static boolean throttleLogging = false;
    public static boolean skipVision = false;
    public static boolean throttleVision = false;
    public static boolean skipDriveLogging = false;
    public static boolean reduceAllocations = false;
    public static boolean skipPowerLogging = false;

    // === Throttle counters ===
    private static int loopCounter = 0;
    private static final int THROTTLE_INTERVAL = 5;

    // === Auto-run state ===
    // Hardcoded ON — experiment starts automatically when robot code runs.
    // Set to false and redeploy when done with experiments.
    private static boolean autoRunEnabled = true;
    private static boolean autoRunStarted = false;
    private static final double PHASE_DURATION_SECONDS = 30.0;
    private static double phaseStartTime = 0;
    private static int currentPhaseIndex = -1; // -1 = not started
    private static boolean autoRunComplete = false;

    /**
     * Each phase: a name (for logging) and a Runnable that sets the flags.
     *
     * PART 1 — Isolation tests: start with ALL optimizations ON (lightest),
     * then for each suspect, turn it OFF (add load), measure, then turn it
     * back ON (restore light load) before testing the next one.
     *
     * PART 2 — Cumulative tests: start with ALL ON again, then progressively
     * turn things OFF one at a time (keeping previous ones off) to find the
     * combination that tips it over.
     */
    private static final Phase[] PHASES = {
        // === PART 1: ISOLATED TESTS (on → off → on for each) ===

        // Establish stable starting point
        new Phase("ALL_ON_INITIAL", LoopExperiments::enableAll),

        // Test 1: DriveLogging (was spiking to 42ms)
        new Phase("TEST_AddDriveLogging", () -> { enableAll(); skipDriveLogging = false; }),
        new Phase("RECOVER_1", LoopExperiments::enableAll),

        // Test 2: Full Vision (was spiking to 41ms)
        new Phase("TEST_AddFullVision", () -> { enableAll(); skipVision = false; throttleVision = false; }),
        new Phase("RECOVER_2", LoopExperiments::enableAll),

        // Test 3: PowerLogging (was spiking to 158ms)
        new Phase("TEST_AddPowerLogging", () -> { enableAll(); skipPowerLogging = false; }),
        new Phase("RECOVER_3", LoopExperiments::enableAll),

        // Test 4: TempReads (8 extra CAN reads)
        new Phase("TEST_AddTempReads", () -> { enableAll(); skipTempReads = false; }),
        new Phase("RECOVER_4", LoopExperiments::enableAll),

        // Test 5: Full-rate Logging (5x more Logger calls)
        new Phase("TEST_AddFullLogging", () -> { enableAll(); throttleLogging = false; }),
        new Phase("RECOVER_5", LoopExperiments::enableAll),

        // Test 6: Allocations (getValue().in() vs getValueAsDouble())
        new Phase("TEST_AddAllocations", () -> { enableAll(); reduceAllocations = false; }),
        new Phase("RECOVER_6", LoopExperiments::enableAll),

        // === PART 2: CUMULATIVE (progressively add load until it breaks) ===

        new Phase("CUMULATIVE_START", LoopExperiments::enableAll),

        // Add them back in order of suspected impact
        new Phase("CUMUL_+DriveLog", () -> {
            enableAll(); skipDriveLogging = false;
        }),
        new Phase("CUMUL_+DriveLog+Vision", () -> {
            enableAll(); skipDriveLogging = false;
            skipVision = false; throttleVision = false;
        }),
        new Phase("CUMUL_+DriveLog+Vision+Power", () -> {
            enableAll(); skipDriveLogging = false;
            skipVision = false; throttleVision = false;
            skipPowerLogging = false;
        }),
        new Phase("CUMUL_+DriveLog+Vision+Power+Temp", () -> {
            enableAll(); skipDriveLogging = false;
            skipVision = false; throttleVision = false;
            skipPowerLogging = false; skipTempReads = false;
        }),
        new Phase("CUMUL_+DriveLog+Vision+Power+Temp+FullLog", () -> {
            enableAll(); skipDriveLogging = false;
            skipVision = false; throttleVision = false;
            skipPowerLogging = false; skipTempReads = false;
            throttleLogging = false;
        }),
        new Phase("CUMUL_ALL_OFF", LoopExperiments::clearAll),
    };

    /** Call once at the start of robotPeriodic. */
    public static void update() {
        loopCounter++;

        // autoRunEnabled is hardcoded — don't read from SmartDashboard
        // (dashboard clients can race and overwrite it to false)

        if (autoRunEnabled && !autoRunComplete) {
            updateAutoRun();
        } else if (!autoRunEnabled) {
            // Manual mode: read toggle states from SmartDashboard
            autoRunStarted = false;
            currentPhaseIndex = -1;
            skipTempReads = SmartDashboard.getBoolean(PREFIX + "SkipTempReads", skipTempReads);
            throttleLogging = SmartDashboard.getBoolean(PREFIX + "ThrottleLogging", throttleLogging);
            skipVision = SmartDashboard.getBoolean(PREFIX + "SkipVision", skipVision);
            throttleVision = SmartDashboard.getBoolean(PREFIX + "ThrottleVision", throttleVision);
            skipDriveLogging = SmartDashboard.getBoolean(PREFIX + "SkipDriveLogging", skipDriveLogging);
            reduceAllocations = SmartDashboard.getBoolean(PREFIX + "ReduceAllocations", reduceAllocations);
            skipPowerLogging = SmartDashboard.getBoolean(PREFIX + "SkipPowerLogging", skipPowerLogging);
        }

        // Always log current state
        String activeFlags =
            (skipTempReads ? "SkipTemp " : "")
            + (throttleLogging ? "ThrottleLog " : "")
            + (skipVision ? "SkipVision " : "")
            + (throttleVision ? "ThrottleVision " : "")
            + (skipDriveLogging ? "SkipDriveLog " : "")
            + (reduceAllocations ? "ReduceAlloc " : "")
            + (skipPowerLogging ? "SkipPower " : "");
        Logger.recordOutput("Experiments/ActiveFlags", activeFlags.isEmpty() ? "NONE" : activeFlags.trim());
        Logger.recordOutput("Experiments/AutoRunActive", autoRunEnabled && !autoRunComplete);

        if (autoRunEnabled && !autoRunComplete && currentPhaseIndex >= 0 && currentPhaseIndex < PHASES.length) {
            double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
            double remaining = PHASE_DURATION_SECONDS - elapsed;
            Logger.recordOutput("Experiments/CurrentPhase", PHASES[currentPhaseIndex].name);
            Logger.recordOutput("Experiments/PhaseIndex", currentPhaseIndex);
            Logger.recordOutput("Experiments/PhaseTimeRemainingS", remaining);
            Logger.recordOutput("Experiments/TotalPhases", PHASES.length);
        } else if (autoRunComplete) {
            Logger.recordOutput("Experiments/CurrentPhase", "COMPLETE");
        }
    }

    private static void updateAutoRun() {
        double now = Timer.getFPGATimestamp();

        if (!autoRunStarted) {
            // Start the sequence
            autoRunStarted = true;
            currentPhaseIndex = 0;
            phaseStartTime = now;
            PHASES[0].apply.run();
            DriverStation.reportWarning("=== EXPERIMENT AUTO-RUN STARTED: " + PHASES[0].name + " ===", false);
            return;
        }

        // Check if current phase is done
        double elapsed = now - phaseStartTime;
        if (elapsed >= PHASE_DURATION_SECONDS) {
            currentPhaseIndex++;
            if (currentPhaseIndex >= PHASES.length) {
                // All phases complete
                clearAll();
                autoRunComplete = true;
                DriverStation.reportWarning("=== EXPERIMENT AUTO-RUN COMPLETE ===", false);
                return;
            }
            // Start next phase
            phaseStartTime = now;
            PHASES[currentPhaseIndex].apply.run();
            DriverStation.reportWarning(
                "=== PHASE " + currentPhaseIndex + "/" + PHASES.length
                + ": " + PHASES[currentPhaseIndex].name + " ===", false);
        }
    }

    /** Publish all toggles to SmartDashboard and apply lightest config at boot. */
    public static void init() {
        // Apply lightest config immediately so the robot is stable from the start
        if (autoRunEnabled) {
            enableAll();
        }

        SmartDashboard.putBoolean(PREFIX + "SkipTempReads", skipTempReads);
        SmartDashboard.putBoolean(PREFIX + "ThrottleLogging", throttleLogging);
        SmartDashboard.putBoolean(PREFIX + "SkipVision", skipVision);
        SmartDashboard.putBoolean(PREFIX + "ThrottleVision", throttleVision);
        SmartDashboard.putBoolean(PREFIX + "SkipDriveLogging", skipDriveLogging);
        SmartDashboard.putBoolean(PREFIX + "ReduceAllocations", reduceAllocations);
        SmartDashboard.putBoolean(PREFIX + "SkipPowerLogging", skipPowerLogging);
        SmartDashboard.putBoolean(PREFIX + "AutoRun", false);
    }

    public static boolean isThrottledCycle() {
        return throttleLogging && (loopCounter % THROTTLE_INTERVAL != 0);
    }

    public static boolean shouldRunVision() {
        if (skipVision) return false;
        if (throttleVision) return (loopCounter % 2 == 0);
        return true;
    }

    public static int getLoopCounter() {
        return loopCounter;
    }

    private static void clearAll() {
        skipTempReads = false;
        throttleLogging = false;
        skipVision = false;
        throttleVision = false;
        skipDriveLogging = false;
        reduceAllocations = false;
        skipPowerLogging = false;
    }

    private static void enableAll() {
        skipTempReads = true;
        throttleLogging = true;
        skipVision = true;
        throttleVision = true;
        skipDriveLogging = true;
        reduceAllocations = true;
        skipPowerLogging = true;
    }

    private static class Phase {
        final String name;
        final Runnable apply;

        Phase(String name, Runnable apply) {
            this.name = name;
            this.apply = apply;
        }
    }
}
