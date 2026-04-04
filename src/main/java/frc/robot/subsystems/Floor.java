package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.sim.SimDeviceRegistrar;
import frc.robot.utils.LoopExperiments;
import frc.robot.utils.LoopTimer;

public class Floor extends SubsystemBase {
    public enum Speed {
        STOP(0),
        FEED(0.95);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    private final TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private double cachedRPM, cachedStatorCurrent, cachedSupplyCurrent, cachedTemp;
    private final LoopTimer loopTimer = new LoopTimer("Timing/Floor");

    public Floor() {
        motor = new TalonFX(Ports.kFloor, Ports.kRoboRioCANBus);

        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true)
            );

        motor.getConfigurator().apply(config);
        SimDeviceRegistrar.registerTalonFX(motor);
        SmartDashboard.putData(this);
    }

    public void set(Speed speed) {
        motor.setControl(
            voltageRequest
                .withOutput(speed.voltage())
        );
    }

    public Command feedCommand() {
        System.out.println("================Floor Feed Command");
        return startEnd(() -> {System.out.println("Starting floor feed"); set(Speed.FEED);}, () -> {System.out.println("stopping floor feed"); set(Speed.STOP);});
    }

    @Override
    public void periodic() {
        loopTimer.startLoop();
        cachedRPM = motor.getVelocity().getValueAsDouble() * 60.0; // always use getValueAsDouble for Floor
        cachedStatorCurrent = motor.getStatorCurrent().getValueAsDouble();
        cachedSupplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
        if (!LoopExperiments.skipTempReads) {
            cachedTemp = motor.getDeviceTemp().getValueAsDouble();
        }

        if (!LoopExperiments.isThrottledCycle()) {
            Logger.recordOutput("Floor/SupplyCurrent", cachedSupplyCurrent);
            if (!LoopExperiments.skipTempReads) {
                Logger.recordOutput("Floor/Temp", cachedTemp);
            }
        }
        loopTimer.endLoop();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("RPM", () -> cachedRPM, null);
        builder.addDoubleProperty("Floor Stator Current", () -> cachedStatorCurrent, null);
        builder.addDoubleProperty("Floor Supply Current", () -> cachedSupplyCurrent, null);
    }
}
