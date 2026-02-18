package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers; // Note your specific path

public class VisionSubsystem extends SubsystemBase {
    private final String[] llNames = {"shooterLL", "backLL"}; //update with actual LL names
    private final String primaryLL = "limelight";

    public VisionSubsystem() {}

    // Get the horizontal offset to the target
    public double getTX() {
        return LimelightHelpers.getTX(primaryLL);
    }

    // Get the vertical offset (useful for distance estimation)
    public double getTY() {
        return LimelightHelpers.getTY(primaryLL);
    }

    // Identify the YOLOv8 class currently being seen
    public String getDetectedClass() {
        return !LimelightHelpers.getNeuralClassID(primaryLL).isEmpty() ?
                LimelightHelpers.getLatestResults(primaryLL).targetingResults.classifierResults.get(0).className :
                "None";
    }

    @Override
    public void periodic() {
        // This runs 50 times a second on the robot
    }

    public List<LimelightHelpers.PoseEstimate> getAllPoseEstimates(double maxOmega) {
        List<LimelightHelpers.PoseEstimate> estimates = new ArrayList<>();
        
        for (String llName : llNames) {
            if (LimelightHelpers.getTV(llName)) { //getTV --> can limelight see a target, if yes go, if not skip
                continue;
            }
            
            LimelightHelpers.PoseEstimate estimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
            
            //add estimates with at least 2 tags when spinning fast
            if (estimate != null && estimate.tagCount > 0) {
                if (maxOmega < 3.0 || estimate.tagCount > 1) {
                    estimates.add(estimate);
                }
            }
        }
        
        return estimates;
    }

    //kalman filter to see how trustworthy the vision code is
    public Matrix<N3, N1> getVisionStdDevsForEstimate(LimelightHelpers.PoseEstimate estimate) { 
        if (estimate == null || estimate.tagCount == 0) { //if 0 tags seen or no pose estimate, IGNORE ALL VISION
            return VecBuilder.fill(1e9, 1e9, 1e9);
        }
        
        double distance = estimate.avgTagDist;
        int tagCount = estimate.tagCount;
        
        //need to tune prolly this is js made up stuff that claude reccomended
        double xyStdDev = 0.05 + (0.02 * distance * distance);
        double thetaStdDev = 0.1 + (0.05 * distance);
        
        if (tagCount == 1) { //if we only see 1 tag trust it less
            xyStdDev *= 2.0;
            thetaStdDev *= 2.5;
        } else if (tagCount >= 5) { //if we see 5 or more (in our alliance zone basically) trust it more
            xyStdDev *= 0.7;
            thetaStdDev *= 0.7;
        }
        
        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    public Pose2d getEstimatedGlobalPose() {
        LimelightHelpers.PoseEstimate bestEstimate = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        
        for (String llName : llNames) {
            if (LimelightHelpers.getTV(llName)) {
                continue; //if not confident skip
            }
            
            LimelightHelpers.PoseEstimate estimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
            
            if (estimate == null || estimate.tagCount == 0) {
                continue;
            }
            
            //more tags = better, closer = better
            double score = estimate.tagCount * 10.0 - estimate.avgTagDist;
            
            if (score > bestScore) {
                bestScore = score;
                bestEstimate = estimate;
            }
        }
        
        return bestEstimate != null ? bestEstimate.pose : null;
    }

    public Double getPoseTimestampSeconds() {
        Pose2d bestPose = getEstimatedGlobalPose();
        if (bestPose == null) { //if not there js ignore the rest
            return null;
        }
        
        //timestamp of best estimate we have
        for (String llName : llNames) {
            LimelightHelpers.PoseEstimate estimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
            
            if (estimate != null && estimate.pose.equals(bestPose)) {
                return estimate.timestampSeconds;
            }
        }
        return null;
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
