package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.simulation.MapleSimSwerveDrivetrain;
import frc.robot.utils.simulation.SimSwerveConstants;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Seconds;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private static final double kSimLoopPeriod = 0.005;
    private Notifier m_simNotifier = null;
    private static double m_lastSimTime;

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

    public Swerve() {
        super(
            TunerConstants.DrivetrainConstants, 
            0,
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.1, 0.1, 0.1),
            getSwerveModuleConstants()
        );

        configureAutoBuilder();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    // Get swerve module constants as an array of constants
    private static SwerveModuleConstants<?, ?, ?>[] getSwerveModuleConstants() {
    SwerveModuleConstants<?, ?, ?>[] modules =
        new SwerveModuleConstants[] {
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        };
    
    // Regulate swerve module constants for simulation if in sim
    if (Utils.isSimulation()) {
        return MapleSimSwerveDrivetrain
            .regulateModuleConstantsForSimulation(modules);
    }

    return modules;
}

    private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the
          // case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                if (!m_hasAppliedOperatorPerspective) {
                    seedFieldCentric();
                }
                m_hasAppliedOperatorPerspective = true;
            });
        }

        if (mapleSimSwerveDrivetrain != null) {
            Pose2d simPose = mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
            super.resetPose(simPose);
            Logger.recordOutput("Drive/Pose", simPose);
          } else {
            Logger.recordOutput("Drive/Pose", getState().Pose);
          }
      
        Logger.recordOutput("BatteryVoltage", RobotController.getBatteryVoltage());
        Logger.recordOutput("Drive/TargetStates", getState().ModuleTargets);
        Logger.recordOutput("Drive/MeasuredStates", getState().ModuleStates);
        Logger.recordOutput("Drive/MeasuredSpeeds", getState().Speeds);
    }

    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    private void startSimThread() {
        mapleSimSwerveDrivetrain =
        new MapleSimSwerveDrivetrain(
            Seconds.of(kSimLoopPeriod),
            SimSwerveConstants.ROBOT_MASS,
            SimSwerveConstants.BUMPER_LENGTH_X,
            SimSwerveConstants.BUMPER_LENGTH_Y,
            SimSwerveConstants.DRIVE_MOTOR_WHEEL,
            SimSwerveConstants.STEER_MOTOR_WHEEL,
            SimSwerveConstants.WHEEL_COF,
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
    m_simNotifier.startPeriodic(kSimLoopPeriod);
    
    // Initialize simulation pose to inside the field on black line for red alliance
    double redAllianceInitialSimX = 5.956;
    double redAllianceInitialSimY = 5.936;

    Pose2d initialSimPose = new Pose2d(redAllianceInitialSimX, redAllianceInitialSimY, new Rotation2d(0));
    mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(initialSimPose);
    super.resetPose(initialSimPose);
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) {
        mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
        // Push one sim update instead of blocking the main loop for 0.1s
        mapleSimSwerveDrivetrain.update();
        }
        super.resetPose(pose);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]t, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}
