package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

/**
 * Lightweight utility for timing sections of the periodic loop.
 * Uses FPGA microsecond timestamps for high-resolution measurements.
 * Results are logged via AdvantageKit in milliseconds.
 */
public class LoopTimer {
    private long sectionStartUs;
    private long loopStartUs;
    private final String prefix;

    public LoopTimer(String prefix) {
        this.prefix = prefix;
    }

    /** Call at the very start of the loop iteration. */
    public void startLoop() {
        loopStartUs = RobotController.getFPGATime();
        sectionStartUs = loopStartUs;
    }

    /** End the current section, log its duration, and start the next section. */
    public void mark(String sectionName) {
        long now = RobotController.getFPGATime();
        double ms = (now - sectionStartUs) / 1000.0;
        Logger.recordOutput(prefix + "/" + sectionName + "Ms", ms);
        sectionStartUs = now;
    }

    /** Call at the end of the loop to log total elapsed time. */
    public void endLoop() {
        long now = RobotController.getFPGATime();
        double totalMs = (now - loopStartUs) / 1000.0;
        Logger.recordOutput(prefix + "/TotalMs", totalMs);
    }
}
