package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Landmarks {
    public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(Inches.of(182.105), Inches.of(158.845));
        }
        return new Translation2d(Inches.of(469.115), Inches.of(158.845));
    }

    public static final AprilTagFieldLayout layout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double fieldLength = layout.getFieldLength();
    public static final double fieldWidth = layout.getFieldWidth();

    //red outpost
    public static final Translation2d redOutpostCenter = getTagPosition(13);
    public static final Translation2d redOutpostOffset = getTagPosition(14);

    //red ds 1
    public static final Translation2d redDSOffset = redOutpostCenter.plus(new Translation2d(0,fieldWidth - 0.5));

    //red hub (backside)
    public static final Translation2d redHub = getTagPosition(3);
    public static final Translation2d redHubOffset = getTagPosition(4);

    //blue outpost
    public static final Translation2d blueOutpostCenter  = getTagPosition(29);
    public static final Translation2d blueOutpostOffset  = getTagPosition(30);

    //blue ds 1
    public static final Translation2d blueDSOffset = blueOutpostCenter.minus(new Translation2d(0, fieldWidth -0.5));

    //blue hub (backside)
    public static final Translation2d blueHub = getTagPosition(3);
    public static final Translation2d blueHubOffset = getTagPosition(4);

    private static Translation2d getTagPosition(int id) {
        Optional<Pose3d> maybePose = layout.getTagPose(id);
        return maybePose
            .map(pose3d -> pose3d.getTranslation().toTranslation2d())
            .orElse(new Translation2d(0, 0));
    }
}
