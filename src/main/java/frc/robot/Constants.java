// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Driving {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

    public static class SimConstants {
        public static final double controllerVelocityCurveExponent = 1.0;
        public static final double controllerRotationCurveExponent = 1.0;
        public static final double controllerDeadbandPercentage = 0.02;
    }

    public static class SimControllerButtons {
        // ZXCV on sim for now
        public static final int kAutoAim = 1;
        public static final int kAutoAlignClimb = 2;
        public static final int kClimb = 3;
        public static final int kUnclimb = 4;

        // TODO: need to bind on actual controller
        public static final int kRobotCentricMode = 5;
        public static final int kAimAndShoot = 6;
        public static final int kShootManually = 7;
        public static final int kIntake = 8;
        public static final int kStowIntake = 9;
        public static final int kHangerUp = 10;
        public static final int kHangerDown = 11;
    }

    public static class SlewLimits {
        public static final LinearAcceleration slewTranslateLimit = MetersPerSecondPerSecond.of(Driving.kMaxSpeed.magnitude() * 10.0);
        public static final AngularAcceleration slewRotateLimit = RadiansPerSecondPerSecond.of(Driving.kMaxRotationalRate.magnitude() * 30.0);
    }
}
