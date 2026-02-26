package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private final String[] llNames = {"limelight-shooter", "limelight-backll"};
    private final String primaryLL = "limelight-shooter";
    private final DoubleSupplier yawSupplier;

    private static final double FORWARD_LL_FORWARD = -0.30;
    private static final double FORWARD_LL_SIDE =  0.00;
    private static final double FORWARD_LL_UP = 0.50;
    private static final double FORWARD_LL_PITCH = 25.0;
    private static final double FORWARD_LL_YAW  =  0.0;  //facing front

    private static final double BACK_LL_FORWARD = -0.40;
    private static final double BACK_LL_SIDE =  0.00;
    private static final double BACK_LL_UP = 0.50;
    private static final double BACK_LL_PITCH = 0.0;
    private static final double BACK_LL_YAW = 180.0; //facing back

    public VisionSubsystem(DoubleSupplier yawSupplier) {
        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-shooter",
            FORWARD_LL_FORWARD, FORWARD_LL_SIDE, FORWARD_LL_UP,
            0.0, FORWARD_LL_PITCH, FORWARD_LL_YAW
        );
        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-backll",
            BACK_LL_FORWARD, BACK_LL_SIDE, BACK_LL_UP,
            0.0, BACK_LL_PITCH, BACK_LL_YAW
        );

        this.yawSupplier = yawSupplier;
    }

    public double getTX() {
        return LimelightHelpers.getTX(primaryLL);
    }

    public double getTY() {
        return LimelightHelpers.getTY(primaryLL);
    }

    // Fixed: was referencing non-existent targetingResults API
    public String getDetectedClass() {
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(primaryLL);
        if (results.targets_Classifier != null && results.targets_Classifier.length > 0) {
            return results.targets_Classifier[0].className;
        }
        return "None";
    }

    @Override
    public void periodic() {
        // MegaTag2 requires you to feed it the robot's current yaw every loop.
        // Replace 0.0 with your gyro yaw (e.g. m_drive.getHeading().getDegrees())
        double robotYawDegrees = yawSupplier.getAsDouble();
        for (String llName : llNames) {
            LimelightHelpers.SetRobotOrientation(llName, robotYawDegrees, 0, 0, 0, 0, 0);
        }
    }

    public List<LimelightHelpers.PoseEstimate> getAllPoseEstimates(double maxOmega) {
        List<LimelightHelpers.PoseEstimate> estimates = new ArrayList<>();

        for (String llName : llNames) {
            // Fixed: was inverted (was skipping cameras that DO see a target)
            if (!LimelightHelpers.getTV(llName)) {
                continue;
            }

            LimelightHelpers.PoseEstimate estimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);

            if (estimate != null && estimate.tagCount > 0) {
                if (maxOmega < 3.0 || estimate.tagCount > 1) {
                    estimates.add(estimate);
                }
            }
        }

        return estimates;
    }

    public Matrix<N3, N1> getVisionStdDevsForEstimate(LimelightHelpers.PoseEstimate estimate) {
        if (estimate == null || estimate.tagCount == 0) {
            return VecBuilder.fill(1e9, 1e9, 1e9);
        }

        double distance = estimate.avgTagDist;
        int tagCount = estimate.tagCount;

        double xyStdDev = 0.05 + (0.02 * distance * distance);
        double thetaStdDev = 0.1 + (0.05 * distance);

        if (tagCount == 1) {
            xyStdDev *= 2.0;
            thetaStdDev *= 2.5;
        } else if (tagCount >= 5) {
            xyStdDev *= 0.7;
            thetaStdDev *= 0.7;
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    public Pose2d getEstimatedGlobalPose() {
        LimelightHelpers.PoseEstimate bestEstimate = null;
        double bestScore = Double.NEGATIVE_INFINITY;

        for (String llName : llNames) {
            // Fixed: was inverted
            if (!LimelightHelpers.getTV(llName)) {
                continue;
            }

            LimelightHelpers.PoseEstimate estimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);

            if (estimate == null || estimate.tagCount == 0) {
                continue;
            }

            double score = estimate.tagCount * 10.0 - estimate.avgTagDist;
            if (score > bestScore) {
                bestScore = score;
                bestEstimate = estimate;
            }
        }

        return bestEstimate != null ? bestEstimate.pose : null;
    }

    // Simplified: previously called getEstimatedGlobalPose() then re-scanned 
    // all cameras to find a matching pose, which was fragile and doing double work
    public Double getPoseTimestampSeconds() {
        LimelightHelpers.PoseEstimate bestEstimate = null;
        double bestScore = Double.NEGATIVE_INFINITY;

        for (String llName : llNames) {
            if (!LimelightHelpers.getTV(llName)) continue;

            LimelightHelpers.PoseEstimate estimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);

            if (estimate == null || estimate.tagCount == 0) continue;

            double score = estimate.tagCount * 10.0 - estimate.avgTagDist;
            if (score > bestScore) {
                bestScore = score;
                bestEstimate = estimate;
            }
        }

        return bestEstimate != null ? bestEstimate.timestampSeconds : null;
    }

    public int getTagCount() {
        int totalTags = 0;
        for (String llName : llNames) {
            LimelightHelpers.PoseEstimate estimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
            if (estimate != null) {
                totalTags += estimate.tagCount;
            }
        }
        return totalTags;
    }
}