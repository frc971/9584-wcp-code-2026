package frc.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N2;

public class DriveInputSmoother {
    private static final double kJoystickDeadband = 0.15;
    private static final double kCurveExponent = 3.0;
    private static final double kTranslationSlewRate = 5.0; // 0 to full in 0.2s
    private static final double kRotationSlewRate = 10.0; // 0 to full in 0.1s

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;
    private final DoubleSupplier rotationInput;
    private final SlewRateLimiter forwardSlewFilter = new SlewRateLimiter(kTranslationSlewRate);
    private final SlewRateLimiter leftSlewFilter = new SlewRateLimiter(kTranslationSlewRate);
    private final SlewRateLimiter rotationSlewFilter = new SlewRateLimiter(kRotationSlewRate);

    public DriveInputSmoother(DoubleSupplier forwardInput, DoubleSupplier leftInput, DoubleSupplier rotationInput) {
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
        this.rotationInput = rotationInput;
    }

    public DriveInputSmoother(DoubleSupplier forwardInput, DoubleSupplier leftInput) {
        this(forwardInput, leftInput, () -> 0);
    }

    public ManualDriveInput getSmoothedInput() {
        final Vector<N2> rawTranslationInput = VecBuilder.fill(forwardInput.getAsDouble(), leftInput.getAsDouble());
        final Vector<N2> deadbandedTranslationInput = MathUtil.applyDeadband(rawTranslationInput, kJoystickDeadband);
        final Vector<N2> curvedTranslationInput = MathUtil.copyDirectionPow(deadbandedTranslationInput, kCurveExponent);

        final double rawRotationInput = rotationInput.getAsDouble();
        final double deadbandedRotationInput = MathUtil.applyDeadband(rawRotationInput, kJoystickDeadband);
        final double curvedRotationInput = MathUtil.copyDirectionPow(deadbandedRotationInput, kCurveExponent);

        return new ManualDriveInput(
            forwardSlewFilter.calculate(curvedTranslationInput.get(0)),
            leftSlewFilter.calculate(curvedTranslationInput.get(1)),
            rotationSlewFilter.calculate(curvedRotationInput)
        );
    }
}
