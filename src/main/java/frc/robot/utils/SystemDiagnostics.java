package frc.robot.utils;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.MemoryUsage;
import java.lang.management.ThreadMXBean;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

/**
 * Logs JVM and system diagnostics every cycle to help diagnose
 * CPU spikes, GC pauses, memory pressure, and NT connection issues.
 *
 * All values are logged under "Diag/" in AdvantageKit.
 */
public class SystemDiagnostics {
    private final MemoryMXBean memoryBean = ManagementFactory.getMemoryMXBean();
    private final ThreadMXBean threadBean = ManagementFactory.getThreadMXBean();

    // Track GC deltas between cycles
    private long prevGcCount = 0;
    private long prevGcTimeMs = 0;

    // Track NT connection count
    private int prevNtConnectionCount = 0;

    // CPU load tracking via FPGA time vs wall time
    private long prevFpgaTimeUs = 0;
    private long prevWallTimeNs = 0;

    // Throttle: only log memory/thread details every Nth cycle to reduce overhead
    private int cycleCount = 0;
    private static final int DETAIL_INTERVAL = 10; // every 10th cycle = 5Hz

    public SystemDiagnostics() {
        prevFpgaTimeUs = RobotController.getFPGATime();
        prevWallTimeNs = System.nanoTime();

        // Sum initial GC stats
        for (GarbageCollectorMXBean gc : ManagementFactory.getGarbageCollectorMXBeans()) {
            prevGcCount += gc.getCollectionCount();
            prevGcTimeMs += gc.getCollectionTime();
        }
    }

    /** Call once per robotPeriodic cycle. */
    public void log() {
        cycleCount++;

        // === GC delta (every cycle — critical for catching the 5s freeze) ===
        long totalGcCount = 0;
        long totalGcTimeMs = 0;
        for (GarbageCollectorMXBean gc : ManagementFactory.getGarbageCollectorMXBeans()) {
            totalGcCount += gc.getCollectionCount();
            totalGcTimeMs += gc.getCollectionTime();
        }
        long deltaGcCount = totalGcCount - prevGcCount;
        long deltaGcTimeMs = totalGcTimeMs - prevGcTimeMs;
        prevGcCount = totalGcCount;
        prevGcTimeMs = totalGcTimeMs;

        Logger.recordOutput("Diag/GC/DeltaCount", deltaGcCount);
        Logger.recordOutput("Diag/GC/DeltaTimeMs", deltaGcTimeMs);
        Logger.recordOutput("Diag/GC/TotalCount", totalGcCount);
        Logger.recordOutput("Diag/GC/TotalTimeMs", totalGcTimeMs);

        // === Heap memory (every cycle — lightweight) ===
        MemoryUsage heap = memoryBean.getHeapMemoryUsage();
        double heapUsedMB = heap.getUsed() / (1024.0 * 1024.0);
        double heapMaxMB = heap.getMax() > 0 ? heap.getMax() / (1024.0 * 1024.0) : -1;
        double heapCommittedMB = heap.getCommitted() / (1024.0 * 1024.0);
        double heapPercent = heapMaxMB > 0 ? (heapUsedMB / heapMaxMB) * 100.0 : -1;

        Logger.recordOutput("Diag/Heap/UsedMB", heapUsedMB);
        Logger.recordOutput("Diag/Heap/CommittedMB", heapCommittedMB);
        Logger.recordOutput("Diag/Heap/MaxMB", heapMaxMB);
        Logger.recordOutput("Diag/Heap/Percent", heapPercent);

        // === NT connections (every cycle — detect connect/disconnect churn) ===
        var connections = NetworkTableInstance.getDefault().getConnections();
        int ntConnectionCount = connections.length;
        Logger.recordOutput("Diag/NT/ConnectionCount", ntConnectionCount);

        // Log connection change events
        if (ntConnectionCount != prevNtConnectionCount) {
            Logger.recordOutput("Diag/NT/ConnectionChange", ntConnectionCount - prevNtConnectionCount);
            // Build a string of connected clients
            StringBuilder clients = new StringBuilder();
            for (var conn : connections) {
                if (clients.length() > 0) clients.append(", ");
                clients.append(conn.remote_id).append("@").append(conn.remote_ip);
            }
            Logger.recordOutput("Diag/NT/ConnectedClients", clients.toString());
            prevNtConnectionCount = ntConnectionCount;
        }

        // === Thread count (throttled) ===
        if (cycleCount % DETAIL_INTERVAL == 0) {
            Logger.recordOutput("Diag/Threads/Count", threadBean.getThreadCount());
            Logger.recordOutput("Diag/Threads/Peak", threadBean.getPeakThreadCount());

            // Non-heap memory (native/JNI — useful for detecting JNI leaks)
            MemoryUsage nonHeap = memoryBean.getNonHeapMemoryUsage();
            Logger.recordOutput("Diag/NonHeap/UsedMB", nonHeap.getUsed() / (1024.0 * 1024.0));
        }

        // === roboRIO CPU load (via /proc/stat on Linux) ===
        // RobotController doesn't expose CPU% directly, but we can estimate
        // user code CPU load from the ratio of UserCodeMS to FullCycleMS
        // (already logged by AdvantageKit). Instead, log FPGA brownout state
        // and system active state which correlate with CPU issues.
        Logger.recordOutput("Diag/FPGA/BrownedOut", RobotController.isBrownedOut());
        Logger.recordOutput("Diag/FPGA/SystemActive", RobotController.isSysActive());
        Logger.recordOutput("Diag/FPGA/Voltage3v3", RobotController.getVoltage3V3());
        Logger.recordOutput("Diag/FPGA/Current3v3", RobotController.getCurrent3V3());
    }
}
