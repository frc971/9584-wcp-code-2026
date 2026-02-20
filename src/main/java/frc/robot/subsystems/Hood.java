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
import com.ctre.phoenix6.controls.VoltageOut;

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
    private final TalonFX motor;
    
    // Control
    private final PIDController pidController;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    
    private double targetPosition = 0.5;

    public Hood() {
        // Initialize Talon FX controller
        motor = new TalonFX(Ports.kHoodMotor);

        // Configure motor
        configureMotor(motor);
        
        // Initialize PID controllers
        pidController = new PIDController(kP, kI, kD);
        
        pidController.setTolerance(kPositionTolerance);
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

    public void setPercentOutput(double percentOutput) {
        motor.setControl(voltageRequest.withOutput(percentOutput));
    }

    public double getPosition() {
        return MathUtil.clamp(motor.getMotorVoltage().getValueAsDouble() / kMaxVoltage, 0.0, 1.0);
    }

    public Command setPosition(double hoodPosition) {
        targetPosition = hoodPosition;
        final double percentOutput = MathUtil.clamp(pidController.calculate(getPosition(), targetPosition), -kMaxOutput, kMaxOutput);

        return Commands.runOnce(() -> setPercentOutput(percentOutput));
    }

    public boolean isPositionWithinTolerance() {
        return pidController.atSetpoint();
    }

    @Override
    public void periodic() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Motor Voltage", () -> motor.getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("Motor Supply Current", () -> motor.getSupplyCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Motor Stator Current", () -> motor.getStatorCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Hood Position", () -> getPosition(), position -> setPosition(position));
    }
}
