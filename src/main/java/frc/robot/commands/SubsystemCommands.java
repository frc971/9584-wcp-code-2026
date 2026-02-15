package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
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
import frc.robot.subsystems.Hanger.Position;

public final class SubsystemCommands {
    private final Swerve swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    private static final double kClimbDriveSpeedMetersPerSecond = 0.35;
    private static final double kClimbDriveDurationSeconds = 0.6;

    private final SwerveRequest.RobotCentric climRobotCentricRequest = new SwerveRequest.RobotCentric();

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
        return Commands.sequence(
            Commands.runOnce(() -> {hanger.positionCommand(Position.HANGER_EXTEND);}),
            Commands.runOnce(() ->{
                swerve.setControl(
                    climRobotCentricRequest
                        .withVelocityX(kClimbDriveSpeedMetersPerSecond)
                        .withVelocityY(0.0)
                        .withRotationalRate(0.0)
                );
            }),
            Commands.runOnce(() -> {hanger.positionCommand(Position.HANGER_HOME);})
        )
        .withTimeout(kClimbDriveDurationSeconds);
    }

    public Command unClimbWithDriveCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {hanger.positionCommand(Position.HANGER_EXTEND);}), //extend hanger
            Commands.runOnce(() ->{
                swerve.setControl(
                    climRobotCentricRequest
                        .withVelocityX(-kClimbDriveSpeedMetersPerSecond) //moving away from tower
                        .withVelocityY(0.0)
                        .withRotationalRate(0.0)
                );
            }),
            Commands.runOnce(() -> {hanger.homingHopperCommand();}) //home hanger (will set to extend hopper)
        )
        .withTimeout(kClimbDriveDurationSeconds);
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
}
