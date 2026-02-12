// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
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
    private final Limelight limelight = new Limelight("limelight");

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(
        Driving.kMaxSpeed.in(MetersPerSecond),
        swerve::getRobotRotation3d
    );
    
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandJoystick simController = new CommandJoystick(2);

    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Driving.kMaxSpeed.in(MetersPerSecond) * Constants.SimConstants.controllerDeadbandPercentage)
        .withRotationalDeadband(Constants.Driving.kMaxRotationalRate.in(RadiansPerSecond) * Constants.SimConstants.controllerDeadbandPercentage);

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

    private final SlewRateLimiter fieldXSlewFilter = new SlewRateLimiter(Constants.SlewLimits.slewTranslateLimit.in(MetersPerSecondPerSecond));
    private final SlewRateLimiter fieldYSlewFilter = new SlewRateLimiter(Constants.SlewLimits.slewTranslateLimit.in(MetersPerSecondPerSecond));
    private final SlewRateLimiter fieldRotateSlewFilter = new SlewRateLimiter(Constants.SlewLimits.slewRotateLimit.in(RadiansPerSecondPerSecond));
    private final SlewRateLimiter robotXSlewFilter = new SlewRateLimiter(Constants.SlewLimits.slewTranslateLimit.in(MetersPerSecondPerSecond));
    private final SlewRateLimiter robotYSlewFilter = new SlewRateLimiter(Constants.SlewLimits.slewTranslateLimit.in(MetersPerSecondPerSecond));
    private final SlewRateLimiter robotRotateSlewFilter = new SlewRateLimiter(Constants.SlewLimits.slewRotateLimit.in(RadiansPerSecondPerSecond));
    private boolean simRobotCentricMode = false;

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

    private final SubsystemCommands simSubsystemCommands = new SubsystemCommands(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        this::getSimForwardInput,
        this::getSimLeftInput
    );

    private SendableChooser<Command> autoChooser;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        limelight.setDefaultCommand(updateVisionCommand());

        if (RobotBase.isReal()){
            configureBindings();
        }
        else{
            configureSimBindings();
        }
        configureAutonomous();
        SmartDashboard.putBoolean("Sim Robot Centric Mode", simRobotCentricMode);
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
    }

    private void configureAutonomous() {
        NamedCommands.registerCommand("Intake", intake.intakeCommand());
        NamedCommands.registerCommand("Aim and Shoot", subsystemCommands.aimAndShoot());
        // Extend the hanger (hooks) to be able to reach the L1 bar
        NamedCommands.registerCommand("Hanger Extend Command", hanger.positionCommand(Hanger.Position.HANGING));
        // Retract the hanger to hook onto the L1 bar
        NamedCommands.registerCommand("Hanger Hook Command", hanger.positionCommand(Hanger.Position.HUNG));

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

        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
            .onTrue(intake.homingCommand())
            .onTrue(hanger.homingCommand());

        driver.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        driver.rightStick().whileTrue(subsystemCommands.autoAim());
        driver.rightBumper().whileTrue(subsystemCommands.shootManually());
        driver.leftTrigger().whileTrue(intake.intakeCommand());
        driver.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));
        driver.start().onTrue(subsystemCommands.autoAlignClimbCommand());

        driver.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        driver.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
    }

    private void configureSimBindings() {
        swerve.setDefaultCommand(
            swerve.applyRequest(() -> {
                if (!isSimControllerConnected() && !isDriverControllerConnected()) {
                    return fieldCentricDrive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0);
                }

                double exponentVelocity =
                    Constants.SimConstants.controllerVelocityCurveExponent;
                double exponentRotation =
                    Constants.SimConstants.controllerRotationCurveExponent;

                if (!simRobotCentricMode) {
                    double fieldX = fieldXSlewFilter.calculate(
                        Constants.Driving.kMaxSpeed.in(MetersPerSecond)
                            * ExponentialConvert(getSimLeftInput(), exponentVelocity));
                    double fieldY = fieldYSlewFilter.calculate(
                        Constants.Driving.kMaxSpeed.in(MetersPerSecond)
                            * ExponentialConvert(getSimForwardInput(), exponentVelocity));
                    double fieldRotate = fieldRotateSlewFilter.calculate(
                        Constants.Driving.kMaxRotationalRate.in(RadiansPerSecond)
                            * ExponentialConvert(getSimRotationInput(), exponentRotation));
                    return fieldCentricDrive.withVelocityX(fieldX).withVelocityY(fieldY).withRotationalRate(fieldRotate);
                } else {
                    double robotX = robotXSlewFilter.calculate(
                        Constants.Driving.kMaxSpeed.in(MetersPerSecond)
                            * ExponentialConvert(getSimLeftInput(), exponentVelocity));
                    double robotY = robotYSlewFilter.calculate(
                        Constants.Driving.kMaxSpeed.in(MetersPerSecond)
                            * ExponentialConvert(getSimForwardInput(), exponentVelocity));
                    double robotRotate = robotRotateSlewFilter.calculate(
                        Constants.Driving.kMaxRotationalRate.in(RadiansPerSecond)
                            * ExponentialConvert(getSimRotationInput(), exponentRotation));
                    return robotCentricDrive.withVelocityX(robotX).withVelocityY(robotY).withRotationalRate(robotRotate);
                }
            }));
        simButton(Constants.SimControllerButtons.kRobotCentricMode)
            .onTrue(Commands.runOnce(this::toggleSimRobotCentricMode));
        // Mirror driver-facing bindings on the sim joystick so the same features exist in sim.
        final Trigger robotCentricModeTrigger = new Trigger(() -> simRobotCentricMode);
        simButton(Constants.SimControllerButtons.kAutoAim)
            .or(driverRightStickButton())
            .or(robotCentricModeTrigger)
            .whileTrue(simSubsystemCommands.autoAim());
        simButton(Constants.SimControllerButtons.kAutoAlignClimb)
            .onTrue(simSubsystemCommands.autoAlignClimbCommand());
        simButton(Constants.SimControllerButtons.kClimb)
            .onTrue(hanger.climbCommand());
        simButton(Constants.SimControllerButtons.kUnclimb)
            .onTrue(hanger.unclimbCommand());
        simButton(Constants.SimControllerButtons.kAimAndShoot)
            .or(driverRightTrigger())
            .whileTrue(simSubsystemCommands.aimAndShoot());
        simButton(Constants.SimControllerButtons.kShootManually)
            .or(driverRightBumper())
            .whileTrue(simSubsystemCommands.shootManually());
        simButton(Constants.SimControllerButtons.kIntake)
            .or(driverLeftTrigger())
            .whileTrue(intake.intakeCommand());
        simButton(Constants.SimControllerButtons.kStowIntake)
            .or(driverLeftBumper())
            .onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));
        simButton(Constants.SimControllerButtons.kHangerUp)
            .or(driverPovUp())
            .onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        simButton(Constants.SimControllerButtons.kHangerDown)
            .or(driverPovDown())
            .onTrue(hanger.positionCommand(Hanger.Position.HUNG));
    }

    private double getSimForwardInput() {
        if (isDriverControllerConnected()) {
            return -driver.getLeftY();
        }
        if (isSimControllerConnected()) {
            return -simController.getRawAxis(1);
        }
        return 0.0;
    }

    private double getSimLeftInput() {
        if (isDriverControllerConnected()) {
            return driver.getLeftX();
        }
        if (isSimControllerConnected()) {
            return simController.getRawAxis(0);
        }
        return 0.0;
    }

    private double getSimRotationInput() {
        if (isDriverControllerConnected()) {
            return -driver.getRightX();
        }
        if (isSimControllerConnected()) {
            return -simController.getRawAxis(2);
        }
        return 0.0;
    }

    private boolean isSimControllerConnected() {
        return DriverStation.isJoystickConnected(simController.getHID().getPort());
    }

    private boolean isDriverControllerConnected() {
        return DriverStation.isJoystickConnected(driver.getHID().getPort());
    }

    private void toggleSimRobotCentricMode() {
        simRobotCentricMode = !simRobotCentricMode;
        SmartDashboard.putBoolean("Sim Robot Centric Mode", simRobotCentricMode);
        DriverStation.reportWarning(
            "Sim Robot Centric Mode: " + (simRobotCentricMode ? "Robot-Centric" : "Field-Centric"),
            false
        );
    }

    private Trigger driverRightTrigger() {
        return new Trigger(() -> isDriverControllerConnected() && driver.getRightTriggerAxis() > 0.25);
    }

    private Trigger driverLeftTrigger() {
        return new Trigger(() -> isDriverControllerConnected() && driver.getLeftTriggerAxis() > 0.25);
    }

    private Trigger driverRightBumper() {
        return new Trigger(() -> isDriverControllerConnected() && driver.getHID().getRawButton(XboxController.Button.kRightBumper.value));
    }

    private Trigger driverLeftBumper() {
        return new Trigger(() -> isDriverControllerConnected() && driver.getHID().getRawButton(XboxController.Button.kLeftBumper.value));
    }

    private Trigger driverRightStickButton() {
        return new Trigger(() -> isDriverControllerConnected() && driver.getHID().getRawButton(XboxController.Button.kRightStick.value));
    }

    private Trigger driverPovUp() {
        return new Trigger(() -> isDriverControllerConnected() && driver.getHID().getPOV() == 0);
    }

    private Trigger driverPovDown() {
        return new Trigger(() -> isDriverControllerConnected() && driver.getHID().getPOV() == 180);
    }

    private boolean isSimButtonAvailable(int buttonNumber) {
        return isSimControllerConnected() && simController.getHID().getButtonCount() >= buttonNumber;
    }

    private boolean isSimButtonPressed(int buttonNumber) {
        return isSimButtonAvailable(buttonNumber) && simController.getHID().getRawButton(buttonNumber);
    }

    private Trigger simButton(int buttonNumber) {
        return new Trigger(() -> isSimButtonPressed(buttonNumber));
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
            final Pose2d currentRobotPose = swerve.getState().Pose;
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        })
        .ignoringDisable(true);
    }

    public static double ExponentialConvert(double controllerValue, double exponent) {
        return Math.copySign(Math.pow(Math.abs(controllerValue), exponent), controllerValue);
    }
    public void autonomousInit() {}
}
