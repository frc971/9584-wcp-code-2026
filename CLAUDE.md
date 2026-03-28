# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FRC Team 9584's 2026 robot code for the WestCoast Products Competitive Concept. Java 17 project using WPILib's command-based framework, CTRE Phoenix 6 for swerve drive, Limelight for vision, and PathPlanner for autonomous paths.

## Build & Development Commands

```bash
./gradlew build              # Compile and run tests
./gradlew test               # Run tests only (JUnit 5)
./gradlew deploy             # Deploy to RoboRIO
./gradlew simulateJava       # Run robot simulation (MapleSim + AdvantageScope)
./gradlew spotlessApply      # Format code (required before commits, CI will reject unformatted code)
```

## Architecture

**Entry point:** `Robot.java` runs periodic loops; `RobotContainer.java` wires together all subsystems, commands, and controller bindings.

**Subsystems** (`src/main/java/frc/robot/subsystems/`):
- `Swerve` - 4-module swerve drivetrain using TalonFX motors and Pigeon2 IMU (config generated in `generated/TunerConstants.java`)
- `Intake`, `Floor`, `Feeder`, `Shooter` - game piece pipeline from intake to scoring
- `Hood` - aiming via PWM servos
- `Hanger` - climbing mechanism
- `Limelight` - vision targeting and field localization

**Commands** (`src/main/java/frc/robot/commands/`):
- `ManualDriveCommand` - teleop driving with slew rate limiting
- `PrepareShotCommand` / `AimAndDriveCommand` - auto-aim workflows
- `SubsystemCommands` - factory methods for common subsystem actions

**Configuration files:**
- `Constants.java` - robot-wide constants (speeds, PID values, field positions)
- `Ports.java` - CAN IDs and PWM port assignments
- `Landmarks.java` - field landmark positions (hub locations per alliance)

**Simulation** (`src/main/java/frc/robot/utils/simulation/`): MapleSim-based physics simulation with `FuelSim` for game piece tracking and `MapleSimSwerveDrivetrain` for drivetrain physics.

**Autonomous paths:** Pre-built in `src/main/deploy/pathplanner/` — 30+ paths and 8 auto routines configured via PathPlanner GUI.

**Utilities** (`src/main/java/frc/util/`): `SwerveTelemetry`, `DriveInputSmoother`, `GeometryUtil`, `ManualDriveInput`, `Stopwatch`.

## Code Conventions

- **Formatter:** Google Java Format, enforced by Spotless. Run `./gradlew spotlessApply` before committing.
- **Naming:** Google Java style — `ALL_CAPS` constants, `camelCase` variables, `PascalCase` classes.
- **Branching:** Trunk-based development. Branch from `main` as `github-username/<issue-number>-short-description`. Keep branches short-lived.
- **PRs:** Require 2 approvals, squash-and-merge into `main`.

## Hardware Reference

- **CAN bus "rio":** RoboRIO bus. **CAN bus "main":** CANivore bus (used by swerve modules).
- **Motors:** TalonFX (Kraken X60) on CAN IDs 9-16. **Servos:** PWM 3-4 for hood.
- **Vision:** Limelight camera named `"limelight"`.

## Key Dependencies

Managed via vendordeps JSON files: Phoenix 6 (CTRE motors), PathPlanner, AdvantageKit (logging), MapleSim (simulation), WPILib NewCommands.
