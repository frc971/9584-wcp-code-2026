package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.Constants;
import frc.robot.Landmarks;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public final class SubsystemCommands {
    private static final double kClimbDriveSpeedMetersPerSecond = 0.35;
    private static final double kClimbDriveDurationSeconds = 0.6;

    private final Swerve swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    private final SwerveRequest.RobotCentric climbRobotRequest = new SwerveRequest.RobotCentric();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    public SubsystemCommands(
        Swerve swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        Swerve swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger
    ) {
        this(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            () -> 0,
            () -> 0
        );
    }

    public Command aimAndShoot() {
        System.out.println("=========Aim and Shoot Command=========");
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);
        return Commands.parallel(
            Commands.print("Aiming and shooting"),
            aimAndDriveCommand,
            Commands.waitSeconds(0.25)
                .andThen(prepareShotCommand),
            Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
                .andThen(feed())
        );
    }

    public Command autoAim() {
        System.out.println("=========Running aim and drive command=========");
        return new AimAndDriveCommand(swerve, forwardInput, leftInput);
    }

    public Command shootManually() {
        System.out.println("========Shooting Manually=========");
        return shooter.dashboardSpinUpCommand()
            .andThen(feed())
            .handleInterrupt(() -> shooter.stop());
    }

    public Command autoAlignClimbCommand() {
        System.out.println("=========Auto Align Climbing=========");
        return Commands.defer(
            () -> AutoBuilder.pathfindToPose(
                Landmarks.climbPose(),
                Constants.ClimbAlignment.kPathConstraints,
                Constants.ClimbAlignment.kGoalEndVelocityMetersPerSecond
            ),
            Set.of(swerve)
        );
    }

    public Command climbWithDriveCommand() {
        System.out.println("=========Climbing=========");
        return Commands.sequence(
            hanger.positionCommand(Hanger.Position.HANGER_EXTEND),
            driveForwardForClimb(),
            hanger.positionCommand(Hanger.Position.HANGER_HOME)
        );
    }

    public Command unclimbWithDriveCommand() {
        System.out.println("=========UnClimbing========");
        return Commands.sequence(
            hanger.positionCommand(Hanger.Position.HANGER_EXTEND),
            driveBackwardForClimb(),
            hanger.positionCommand(Hanger.Position.HANGER_HOME)
        );
    }
    
    private Command feed() {
        System.out.println("=========Feed========");
        return Commands.sequence(
            Commands.waitSeconds(0.25),
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.125)
                    .andThen(floor.feedCommand().alongWith(intake.agitateCommand()))
            )
        );
    }

    private Command driveForwardForClimb() {
        return driveRobotForClimb(kClimbDriveSpeedMetersPerSecond);
    }

    private Command driveBackwardForClimb() {
        return driveRobotForClimb(-kClimbDriveSpeedMetersPerSecond);
    }

    private Command driveRobotForClimb(double velocityXMetersPerSecond) {
        return Commands.runEnd(
            () -> swerve.setControl(
                climbRobotRequest
                    .withVelocityX(velocityXMetersPerSecond)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)
            ),
            () -> swerve.setControl(idleRequest),
            swerve
        ).withTimeout(kClimbDriveDurationSeconds);
    }
}
