// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
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
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.simulation.Dimensions;
import frc.robot.utils.simulation.FuelSim;
import frc.util.SwerveTelemetry;

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

    private ManualDriveCommand manualDriveCommand;

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
        if (RobotBase.isSimulation()) {
            configureFuelSim();
        }
        SmartDashboard.putBoolean("Sim Robot Centric Mode", simRobotCentricMode);
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
    }

    private void configureAutonomous() {
        NamedCommands.registerCommand("Intake", intake.intakeCommand());
        NamedCommands.registerCommand("Aim and Shoot", subsystemCommands.aimAndShoot());
        // Extend the hanger (hooks) to be able to reach the L1 bar
        NamedCommands.registerCommand("Hanger Extend Command", hanger.positionCommand(Hanger.Position.HANGER_EXTEND));
        // Retract the hanger to hook onto the L1 bar
        NamedCommands.registerCommand("Hanger Hook Command", hanger.positionCommand(Hanger.Position.HANGER_HOME));

        autoChooser = AutoBuilder.buildAutoChooser("Left Neutral Stage Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureFuelSim() {
        FuelSim instance = FuelSim.getInstance();
        instance.clearFuel();
        instance.registerRobot(
            Dimensions.FULL_WIDTH,
            Dimensions.FULL_LENGTH,
            Dimensions.BUMPER_HEIGHT,
            () -> swerve.getState().Pose,
            this::getFieldRelativeChassisSpeedsForSim
        );
        instance.registerIntake(
            -Dimensions.FULL_LENGTH / 2.0,
            Dimensions.FULL_LENGTH / 2.0,
            -Dimensions.FULL_WIDTH / 2.0,
            Dimensions.FULL_WIDTH / 2.0,
            intake::isIntaking,
            () -> Logger.recordOutput("FuelSim/LastEvent", "Intake")
        );

        instance.spawnStartingFuel();
        instance.start();

        Command spawnFuelCommand = Commands.runOnce(this::spawnFuelInFrontOfRobot)
            .ignoringDisable(true)
            .withName("FuelSim/Spawn Fuel");
        Command resetFuelCommand = Commands.runOnce(() -> {
                instance.clearFuel();
                instance.spawnStartingFuel();
                Logger.recordOutput("FuelSim/LastEvent", "Reset");
            })
            .ignoringDisable(true)
            .withName("FuelSim/Reset Fuel");
        Command launchFuelCommand = Commands.runOnce(() -> launchFuelInSim(MetersPerSecond.of(8), Degrees.of(45)))
            .ignoringDisable(true)
            .withName("FuelSim/Launch Fuel");

        SmartDashboard.putData(spawnFuelCommand);
        SmartDashboard.putData(resetFuelCommand);
        SmartDashboard.putData(launchFuelCommand);
    }

    public void resetFuelSim() {
        if (!RobotBase.isSimulation()) {
            return;
        }
        FuelSim instance = FuelSim.getInstance();
        instance.clearFuel();
        instance.spawnStartingFuel();
        Logger.recordOutput("FuelSim/LastEvent", "Auto Reset");
    }

    private void spawnFuelInFrontOfRobot() {
        Pose2d pose = swerve.getState().Pose;
        Translation2d offset = new Translation2d(Dimensions.FULL_LENGTH / 2.0 + 0.1, 0)
            .rotateBy(pose.getRotation());
        Translation3d location = new Translation3d(
            pose.getX() + offset.getX(),
            pose.getY() + offset.getY(),
            Dimensions.BUMPER_HEIGHT / 2.0
        );
        FuelSim.getInstance().spawnFuel(location, new Translation3d());
        Logger.recordOutput("FuelSim/LastEvent", "Manual Spawn");
    }

    private void launchFuelInSim(LinearVelocity velocity, Angle elevation) {
        Pose2d pose = swerve.getState().Pose;
        Translation2d muzzleOffset = new Translation2d(Dimensions.FULL_LENGTH / 2.0, 0)
            .rotateBy(pose.getRotation());
        Translation3d initialPosition = new Translation3d(
            pose.getX() + muzzleOffset.getX(),
            pose.getY() + muzzleOffset.getY(),
            Dimensions.BUMPER_HEIGHT + 0.25
        );
        Translation3d launchVelocity = createLaunchVelocity(velocity, elevation, pose.getRotation());
        FuelSim.getInstance().spawnFuel(initialPosition, launchVelocity);
        Logger.recordOutput("FuelSim/LastEvent", "Launch");
    }

    private Translation3d createLaunchVelocity(LinearVelocity velocity, Angle elevation, Rotation2d heading) {
        double speed = velocity.in(MetersPerSecond);
        double elevationRadians = elevation.in(Radians);
        double planarSpeed = speed * Math.cos(elevationRadians);
        double verticalSpeed = speed * Math.sin(elevationRadians);
        Translation2d planar = new Translation2d(planarSpeed, 0).rotateBy(heading);
        return new Translation3d(planar.getX(), planar.getY(), verticalSpeed);
    }

    private ChassisSpeeds getFieldRelativeChassisSpeedsForSim() {
        ChassisSpeeds speeds = swerve.getState().Speeds;
        if (speeds == null) {
            return new ChassisSpeeds();
        }
        return new ChassisSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond
        );
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
            .onTrue(intake.homingCommand());
        RobotModeTriggers.autonomous()
            .onTrue(hanger.homingHopperCommand());
        //when switching from auto to teleop, extend from tower to ground and move away from tower
        RobotModeTriggers.teleop()
            .onTrue(Commands.runOnce(() -> subsystemCommands.unClimbWithDriveCommand()));

        driverLeftTrigger().whileTrue(intake.intakeCommand());
        driverLeftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));

        driverRightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        driverRightBumper().whileTrue(subsystemCommands.shootManually());

        driverRightStickButton().whileTrue(subsystemCommands.autoAim());
        driverLeftStickButton().onTrue(subsystemCommands.autoAlignClimbCommand());
        driverBButton().onTrue(Commands.runOnce(() -> {
            if (manualDriveCommand != null) {
                manualDriveCommand.toggleRobotCentricMode();
            }
        }));

        driverPovUp().onTrue(hanger.climbCommand());
        driverPovDown().onTrue(hanger.unclimbCommand());
        driverPovLeft().onTrue(hanger.positionCommand(Hanger.Position.HANGER_EXTEND));
        driverPovRight().onTrue(hanger.positionCommand(Hanger.Position.HANGER_HOME));
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
            .or(driverBButton())
            .onTrue(Commands.runOnce(this::toggleSimRobotCentricMode));
        // Mirror driver-facing bindings on the sim joystick so the same features exist in sim.
        simButton(Constants.SimControllerButtons.kAutoAim)
            .or(driverRightStickButton())
            .whileTrue(simSubsystemCommands.autoAim());
        simButton(Constants.SimControllerButtons.kAutoAlignClimb)
            .or(driverLeftStickButton())
            .onTrue(simSubsystemCommands.climbWithAutoAlign());
        simButton(Constants.SimControllerButtons.kClimb)
            .or(driverPovUp())
            .onTrue(simSubsystemCommands.climbWithDriveCommand());
        simButton(Constants.SimControllerButtons.kUnclimb)
            .or(driverPovDown())
            .onTrue(simSubsystemCommands.unClimbWithDriveCommand());
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
            .or(driverPovLeft())
            .onTrue(hanger.positionCommand(Hanger.Position.HANGER_EXTEND));
        simButton(Constants.SimControllerButtons.kHangerDown)
            .or(driverPovRight())
            .onTrue(hanger.positionCommand(Hanger.Position.HANGER_HOME));
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

    private Trigger driverLeftStickButton() {
        return new Trigger(() -> isDriverControllerConnected() && driver.getHID().getRawButton(XboxController.Button.kLeftStick.value));
    }

    private Trigger driverBButton() {
        return new Trigger(() -> isDriverControllerConnected() && driver.getHID().getRawButton(XboxController.Button.kB.value));
    }

    private Trigger driverPovUp() {
        return new Trigger(() -> isDriverControllerConnected() && driver.getHID().getPOV() == 0);
    }

    private Trigger driverPovDown() {
        return new Trigger(() -> isDriverControllerConnected() && driver.getHID().getPOV() == 180);
    }

    private Trigger driverPovLeft() {
        return new Trigger(() -> isDriverControllerConnected() && driver.getHID().getPOV() == 270);
    }

    private Trigger driverPovRight() {
        return new Trigger(() -> isDriverControllerConnected() && driver.getHID().getPOV() == 90);
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
        manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -driver.getLeftY(), 
            () -> -driver.getLeftX(), 
            () -> -driver.getRightX()
        );
        swerve.setDefaultCommand(manualDriveCommand);
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
