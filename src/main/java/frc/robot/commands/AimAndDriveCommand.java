package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.Landmarks;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;
import edu.wpi.first.wpilibj.DriverStation;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;
    private static final double kPoseEdgeMarginMeters = 0.3;
    private PoseInvalidReason lastPoseWarningReason = null;
    private Pose2d lastUsablePose = null;

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Driving.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);

    public AimAndDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve);
    }

    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        final Pose2d poseForTargeting = getCachedPoseForAiming();
        if (poseForTargeting == null) {
            return false;
        }
        final Rotation2d targetHeading = getTargetHeadingInOperatorPerspective(poseForTargeting);
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.minus(swerve.getOperatorForwardDirection());
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    private Rotation2d getTargetHeadingInOperatorPerspective(Pose2d poseForTargeting) {
        return getTargetHeadingInFieldFrame(poseForTargeting).minus(swerve.getOperatorForwardDirection());
    }

    private Rotation2d getTargetHeadingInFieldFrame(Pose2d poseForTargeting) {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = poseForTargeting.getTranslation();
        return hubPosition.minus(robotPosition).getAngle();
    }

    @Override
    public void initialize() {
        lastPoseWarningReason = null;
        lastUsablePose = null;
    }

    @Override
    public void execute() {
        final Pose2d poseForTargeting = getPoseForTargeting();
        if (poseForTargeting == null) {
            swerve.requestIdle();
            return;
        }
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        final Rotation2d targetHeading = getTargetHeadingInFieldFrame(poseForTargeting);
        swerve.setControl(
            fieldCentricFacingAngleRequest
                .withVelocityX(Driving.kMaxSpeed.times(input.forward))
                .withVelocityY(Driving.kMaxSpeed.times(input.left))
                .withTargetDirection(targetHeading)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private Pose2d getPoseForTargeting() {
        final Pose2d currentPose = swerve.getState().Pose;
        final PoseValidationResult validation = validatePose(currentPose);
        if (validation.valid()) {
            lastPoseWarningReason = null;
            lastUsablePose = currentPose;
            return currentPose;
        }

        emitPoseWarning(validation);

        Pose2d fallback = null;
        if (validation.reason() == PoseInvalidReason.OUTSIDE_FIELD && currentPose != null) {
            final Translation2d translation = currentPose.getTranslation();
            final double x = translation.getX();
            final double y = translation.getY();
            if (Double.isFinite(x) && Double.isFinite(y)) {
                fallback = clampPoseToField(currentPose);
            }
        }

        if (fallback == null) {
            fallback = lastUsablePose;
        }

        if (fallback != null) {
            lastUsablePose = fallback;
        }
        return fallback;
    }

    private Pose2d getCachedPoseForAiming() {
        final Pose2d currentPose = swerve.getState().Pose;
        final PoseValidationResult validation = validatePose(currentPose);
        if (validation.valid()) {
            return currentPose;
        }
        return lastUsablePose;
    }

    private Pose2d clampPoseToField(Pose2d pose) {
        final Translation2d translation = pose.getTranslation();
        final double clampedX = MathUtil.clamp(translation.getX(), 0.0, Landmarks.fieldLength);
        final double clampedY = MathUtil.clamp(translation.getY(), 0.0, Landmarks.fieldWidth);
        if (clampedX == translation.getX() && clampedY == translation.getY()) {
            return pose;
        }
        return new Pose2d(clampedX, clampedY, pose.getRotation());
    }

    private PoseValidationResult validatePose(Pose2d pose) {
        if (pose == null) {
            return new PoseValidationResult(false, PoseInvalidReason.NULL_POSE, Double.NaN, Double.NaN);
        }
        final Translation2d translation = pose.getTranslation();
        final double x = translation.getX();
        final double y = translation.getY();
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            return new PoseValidationResult(false, PoseInvalidReason.NON_FINITE_VALUES, x, y);
        }
        final boolean insideField = x >= -kPoseEdgeMarginMeters
            && x <= Landmarks.fieldLength + kPoseEdgeMarginMeters
            && y >= -kPoseEdgeMarginMeters
            && y <= Landmarks.fieldWidth + kPoseEdgeMarginMeters;
        if (!insideField) {
            return new PoseValidationResult(false, PoseInvalidReason.OUTSIDE_FIELD, x, y);
        }
        return new PoseValidationResult(true, null, x, y);
    }

    private void emitPoseWarning(PoseValidationResult validation) {
        if (validation.reason() == null || validation.reason() == lastPoseWarningReason) {
            return;
        }
        DriverStation.reportWarning(buildPoseWarningMessage(validation), false);
        lastPoseWarningReason = validation.reason();
    }

    private String buildPoseWarningMessage(PoseValidationResult validation) {
        return String.format(
            "Auto aim limited (%s). Pose=(%s m, %s m), allowed X [%.2f, %.2f], allowed Y [%.2f, %.2f]",
            validation.reason().description,
            formatMeters(validation.xMeters()),
            formatMeters(validation.yMeters()),
            -kPoseEdgeMarginMeters,
            Landmarks.fieldLength + kPoseEdgeMarginMeters,
            -kPoseEdgeMarginMeters,
            Landmarks.fieldWidth + kPoseEdgeMarginMeters
        );
    }

    private String formatMeters(double value) {
        return Double.isFinite(value) ? String.format("%.2f", value) : "NaN";
    }

    private record PoseValidationResult(boolean valid, PoseInvalidReason reason, double xMeters, double yMeters) { }

    private enum PoseInvalidReason {
        NULL_POSE("pose not yet available"),
        NON_FINITE_VALUES("pose contains non-finite values"),
        OUTSIDE_FIELD("pose translation outside field bounds");

        private final String description;

        PoseInvalidReason(String description) {
            this.description = description;
        }
    }
}
