package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

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
    private static final double kPoseEdgeMarginMeters = 0.1;
    private boolean poseWarningIssued = false;

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
        if (!currentPoseIsValid()) {
            return false;
        }
        final Rotation2d targetHeading = getTargetHeadingInOperatorPerspective();
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.minus(swerve.getOperatorForwardDirection());
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    private Rotation2d getTargetHeadingInOperatorPerspective() {
        return getTargetHeadingInFieldFrame().minus(swerve.getOperatorForwardDirection());
    }

    private Rotation2d getTargetHeadingInFieldFrame() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        return hubPosition.minus(robotPosition).getAngle();
    }

    private boolean isPoseValid(Pose2d pose) {
        if (pose == null) {
            return false;
        }
        final Translation2d translation = pose.getTranslation();
        final double x = translation.getX();
        final double y = translation.getY();
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            return false;
        }
        return x >= -kPoseEdgeMarginMeters
            && x <= Landmarks.fieldLength + kPoseEdgeMarginMeters
            && y >= -kPoseEdgeMarginMeters
            && y <= Landmarks.fieldWidth + kPoseEdgeMarginMeters;
    }

    private boolean currentPoseIsValid() {
        return isPoseValid(swerve.getState().Pose);
    }

    @Override
    public void initialize() {
        poseWarningIssued = false;
        ensurePoseValidWithWarning();
    }

    @Override
    public void execute() {
        if (!ensurePoseValidWithWarning()) {
            swerve.requestIdle();
            return;
        }
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        final Rotation2d targetHeading = getTargetHeadingInOperatorPerspective();
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

    private boolean ensurePoseValidWithWarning() {
        final boolean valid = currentPoseIsValid();
        if (!valid) {
            if (!poseWarningIssued) {
                DriverStation.reportWarning("Auto aim blocked: robot pose outside field bounds", false);
                poseWarningIssued = true;
            }
        } else {
            poseWarningIssued = false;
        }
        return valid;
    }
}
