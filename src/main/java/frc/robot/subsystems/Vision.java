package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * This class manages all Limelight cameras on the robot.
 * It uses MegaTag 2 to fuse vision with the Swerve's gyro.
 */
public class Vision extends SubsystemBase {
    // Hostnames of your Limelights (e.g., "limelight-front", "limelight-back")
    private final String[] cameraNames;

    public Vision(String... names) {
        this.cameraNames = names;
    }

    /**
     * Polls all cameras and calculates vision measurements.
     * @param robotRotation The current rotation of the robot (from Swerve).
     */
    public List<Measurement> getMeasurements(Pose2d currentPose) {
        List<Measurement> results = new ArrayList<>();

        for (String name : cameraNames) {
            // 1. Tell Limelight where we are pointing (Required for MegaTag 2)
            LimelightHelpers.SetRobotOrientation(name, currentPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

            // 2. Get the MegaTag 2 estimate from this camera
            PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

            // 3. Filter: Only use data if tags are visible and close enough (under 5 meters)
            if (mt2 != null && mt2.tagCount > 0 && mt2.avgTagDist < 5.0) {

                // 4. Calculate Trust: More tags = More trust. More distance = Less trust.
                double trustScale = mt2.avgTagDist * (1.0 / mt2.tagCount);
                double xyStdDev = 0.1 * trustScale;

                // 5. Build measurement. Note: Rotation StdDev is 999999 (ignore vision rotation)
                results.add(new Measurement(
                        mt2,
                        VecBuilder.fill(xyStdDev, xyStdDev, 999999)
                ));
            }
        }
        return results;
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> stdDevs;

        public Measurement(PoseEstimate mt2, Matrix<N3, N1> stdDevs) {
            this.poseEstimate = mt2;
            this.stdDevs = stdDevs;
        }
    }
}
