package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.controller.PIDController;

public class Hood extends SubsystemBase {
    // Position limits (0.0 to 1.0 based on analog voltage)
    private static final double kMinPosition = 0.01;
    private static final double kMaxPosition = 0.77;
    private static final double kPositionTolerance = 0.01;
    
    // PID constants - TUNE THESE through testing!
    private static final double kP = 3.0;
    private static final double kI = 0.0;
    private static final double kD = 0.1;

    private static final double kMaxVoltage = 5.0;
    
    // Max output to motors
    private static final double kMaxOutput = 0.5;
    
    // Hardware
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final AnalogInput leftFeedback;
    private final AnalogInput rightFeedback;
    
    // Control
    private final PIDController leftPID;
    private final PIDController rightPID;
    private final DutyCycleOut leftControlRequest = new DutyCycleOut(0);
    private final DutyCycleOut rightControlRequest = new DutyCycleOut(0);
    
    private double targetPosition = 0.5;

    public Hood() {
        // Initialize Talon FX controllers
        leftMotor = new TalonFX(Ports.kHoodLeftMotor);
        rightMotor = new TalonFX(Ports.kHoodRightMotor);
        
        // Configure motors
        configureMotor(leftMotor);
        configureMotor(rightMotor);
        
        // Initialize analog inputs for position feedback
        leftFeedback = new AnalogInput(Ports.kHoodLeftFeedback);
        rightFeedback = new AnalogInput(Ports.kHoodRightFeedback);
        
        // Initialize PID controllers
        leftPID = new PIDController(kP, kI, kD);
        rightPID = new PIDController(kP, kI, kD);
        
        leftPID.setTolerance(kPositionTolerance);
        rightPID.setTolerance(kPositionTolerance);
        
        // Set initial position
        setPosition(targetPosition);
    }

    private void configureMotor(TalonFX motor) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(2))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(2))
                    .withSupplyCurrentLimitEnable(true)
            );
        motor.getConfigurator().apply(config);
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
        targetPosition = clampedPosition;
    }

    /** Expects a position between 0.0 and 1.0 */
    public Command positionCommand(double position) {
        return runOnce(() -> setPosition(position))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public boolean isPositionWithinTolerance() {
        return leftPID.atSetpoint() && rightPID.atSetpoint();
    }

    private void updateCurrentPosition() {
        double leftPercentOutput = leftPID.calculate(getLeftPosition(), targetPosition);
        double rightPercentOutput = rightPID.calculate(getRightPosition(), targetPosition);

        leftPercentOutput = MathUtil.clamp(leftPercentOutput, -kMaxOutput, kMaxOutput);
        rightPercentOutput = MathUtil.clamp(rightPercentOutput, -kMaxOutput, kMaxOutput);

        leftMotor.setControl(leftControlRequest.withOutput(leftPercentOutput));
        rightMotor.setControl(rightControlRequest.withOutput(rightPercentOutput));
    }

    public double getLeftPosition() {
        return MathUtil.clamp(leftFeedback.getVoltage() / kMaxVoltage, 0.0, 1.0);
    }

    public double getRightPosition() {
        return MathUtil.clamp(rightFeedback.getVoltage() / kMaxVoltage, 0.0, 1.0);
    }

    @Override
    public void periodic() {
        updateCurrentPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Current Left Position", () -> getLeftPosition(), null);
        builder.addDoubleProperty("Current Right Position", () -> getRightPosition(), null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, value -> setPosition(value));
    }
}
