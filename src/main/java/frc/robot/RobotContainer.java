// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.util.SwerveTelemetry;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
    // Add all cameras here: e.g.: new Vision("limelight-front", "limelight-back", ...)
    private final Vision limelight = new Vision("limelight");

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    
    private final CommandXboxController driver = new CommandXboxController(0);
    
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX()
    );

    private SendableChooser<Command> autoChooser;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        configureAutonomous();
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
    }

    private void configureAutonomous() {
        //NamedCommands.registerCommand("Intake Fuel", autoCommands.AutoIntakeFuel());

        autoChooser = AutoBuilder.buildAutoChooser("Left Neutral Stage Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
        if (selected == null) {
            DriverStation.reportWarning("No autonomous command selected", false);
            return Commands.none();
        }
        return selected;
    }
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        configureManualDriveBindings();
        limelight.setDefaultCommand(updateVisionCommand());

        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
            .onTrue(intake.homingCommand())
            .onTrue(hanger.homingCommand());

        driver.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        driver.rightBumper().whileTrue(subsystemCommands.shootManually());
        driver.leftTrigger().whileTrue(intake.intakeCommand());
        driver.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));

        driver.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        driver.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -driver.getLeftY(), 
            () -> -driver.getLeftX(), 
            () -> -driver.getRightX()
        );
        swerve.setDefaultCommand(manualDriveCommand);
        driver.a().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
        driver.b().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
        driver.x().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));
        driver.y().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero)));
        driver.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));
    }

    private Command updateVisionCommand() {
        return limelight.run(() -> {
                    // Get current pose from CTRE Swerve
                    final Pose2d currentRobotPose = swerve.getState().Pose;

                    // Get all valid camera updates
                    var measurements = limelight.getMeasurements(currentRobotPose);

                    // Apply each update to the CTRE Pose Estimator
                    for (Vision.Measurement m : measurements) {
                        swerve.addVisionMeasurement(
                                m.poseEstimate.pose,
                                m.poseEstimate.timestampSeconds,
                                m.stdDevs
                        );
                    }
                })
                .ignoringDisable(true); // Keep tracking while disabled so auto starts aligned!
    }
    public void autonomousInit() {}
}
