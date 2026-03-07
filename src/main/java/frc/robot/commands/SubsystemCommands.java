package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.Constants;
import frc.robot.Landmarks;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public final class SubsystemCommands {
    private final CommandSwerveDrivetrain swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(
        CommandSwerveDrivetrain swerve,
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
        CommandSwerveDrivetrain swerve,
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
        return
            Commands.print("Shooting manually") 
            .andThen(shooter.dashboardSpinUpCommand())
            .andThen(Commands.print("Done spinning shooter"))
            .andThen(feed())
            .andThen(Commands.print("Done feeding"))
            .handleInterrupt(() -> shooter.stop());
    }

    public Command shootManualForShootAuto(Supplier<Pose2d> autoStartPoseSupplier) {
        final double kMinAutoDisplacement = 0.5; // meters
        System.out.println("========Shooting Manually (Shoot Auto)=========");
        return Commands.sequence(
            Commands.runOnce(() -> {
                Logger.recordOutput("Auto/ShootPhaseStartPose", swerve.getState().Pose);
                Logger.recordOutput("Auto/ShootPhaseSpeeds", swerve.getState().Speeds);
            }),
            Commands.print("Shoot Auto: Spinning up to 3000 RPM"),
            shooter.spinUpCommand(3000.0),
            Commands.print("Shoot Auto: Shooter at speed, checking displacement"),
            Commands.either(
                feed().beforeStarting(() -> {
                    Logger.recordOutput("Auto/FeedAllowed", true);
                    System.out.println("Shoot Auto: Displacement OK, proceeding to feed");
                }),
                Commands.runOnce(() -> {
                    Pose2d startPose = autoStartPoseSupplier.get();
                    Pose2d currentPose = swerve.getState().Pose;
                    double disp = startPose != null
                        ? currentPose.getTranslation().getDistance(startPose.getTranslation())
                        : -1;
                    String msg = "AUTO SAFETY: Robot displacement " + String.format("%.3f", disp)
                        + "m < " + kMinAutoDisplacement + "m minimum. "
                        + "Feed SKIPPED to protect intake. "
                        + "Start=" + startPose + " Current=" + currentPose;
                    System.out.println(msg);
                    DriverStation.reportWarning(msg, false);
                    Logger.recordOutput("Auto/SafetyTriggered", true);
                    Logger.recordOutput("Auto/DisplacementWhenBlocked", disp);
                    shooter.stop();
                }),
                () -> {
                    Pose2d startPose = autoStartPoseSupplier.get();
                    if (startPose == null) {
                        // No start pose recorded — fail-open so other autos are unaffected
                        return true;
                    }
                    Pose2d currentPose = swerve.getState().Pose;
                    double disp = currentPose.getTranslation().getDistance(
                        startPose.getTranslation());
                    Logger.recordOutput("Auto/DisplacementAtFeedCheck", disp);
                    if (disp < kMinAutoDisplacement) {
                        System.out.println("Displacement check FAILED: "
                            + String.format("%.3f", disp) + "m < " + kMinAutoDisplacement + "m");
                    }
                    return disp >= kMinAutoDisplacement;
                }
            )
        ).handleInterrupt(() -> shooter.stop());
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
