# AdvantageScope Setup & Usage Guide

Guide for using AdvantageScope to view and debug robot pose/localization data.

## Installation (macOS)

1. Download from [GitHub Releases](https://github.com/Mechanical-Advantage/AdvantageScope/releases/latest):
   - Apple Silicon (M1/M2/M3/M4): `advantagescope-mac-arm64-v26.x.x.dmg`
   - Intel: `advantagescope-mac-x64-v26.x.x.dmg`
   - Or via Homebrew: `brew install --cask advantagescope`
2. Open the `.dmg` and drag AdvantageScope to Applications.
3. If macOS blocks the first launch, go to **System Settings > Privacy & Security** and click "Open Anyway."

## Connecting to the Robot (Real-Time)

The robot publishes all `Logger.recordOutput()` data to NetworkTables via `NT4Publisher` (configured in `Robot.java`).

1. Connect your laptop to the robot's WiFi network, or USB-tether to the RoboRIO.
2. Open AdvantageScope.
3. Go to **AdvantageScope > Settings** and set the robot address:
   - WiFi/Ethernet: `10.95.84.2`
   - USB: `172.22.11.2`
4. Go to **File > Connect to Robot**.
5. The title bar shows "Searching..." until connected. Data streams automatically once connected.

## Viewing Robot Pose on a 2D Field

1. Click the **+** tab button and select **2D Field**.
2. In the left sidebar, expand the data tree. Key paths:
   - `AdvantageKit/RealOutputs/Drive/Pose` — fused odometry + vision pose
   - `AdvantageKit/RealOutputs/Vision/Poses` — raw Limelight pose estimates
3. **Drag** `Drive/Pose` into the "Poses" area of the 2D Field tab. A robot icon appears on the field.
4. **Drag** `Vision/Poses` into the same area. Right-click to set a different color.
5. Any disagreement between the two shows vision accuracy issues at a glance.

## Debugging Vision Localization

Add a **Line Graph** tab (click **+**) and drag these fields onto it:

| Field | What It Shows |
|-------|---------------|
| `Vision/HeadingErrorDeg` | Rotation difference between vision and odometry (most important for diagnosing heading drift) |
| `Vision/TranslationErrorM` | Position difference between vision and odometry |
| `Vision/TagCounts` | Number of AprilTags seen per estimate |
| `Vision/AvgTagDists` | Distance to tags (further = less accurate) |
| `Vision/StdDevsXY` | Trust weight applied to x/y position |
| `Vision/StdDevsTheta` | Trust weight applied to heading |
| `Vision/Latencies` | Pipeline latency per estimate |
| `Vision/OmegaRadPerSec` | Robot angular velocity (estimates are filtered when spinning fast) |
| `Vision/EstimateCount` | Number of estimates accepted per cycle (0 = no tags seen or filtered out) |

### What to look for

- **HeadingErrorDeg growing over time** — vision is fighting the gyro (likely theta std dev is too low for MegaTag2).
- **Vision/Poses jumping around while Drive/Pose is smooth** — noisy vision data, check camera mounting and tag distance.
- **EstimateCount stuck at 0** — Limelight isn't seeing tags. Check camera pipeline, field of view, and lighting.
- **Large TranslationErrorM** — possible camera-to-robot transform misconfiguration in the Limelight web UI.

## Replaying Log Files

The robot writes `.wpilog` files to a USB drive on the RoboRIO at `/media/sda1/`. **A USB drive must be plugged into the RoboRIO for file logging to work.**

### Downloading logs from the robot

1. Connect to the robot (WiFi or USB).
2. Go to **File > Download Logs...**
3. Select the log files you want and choose a destination folder.

### Opening log files

- **File > Open Log(s)...** and browse to the `.wpilog` file, or
- **Drag and drop** the file onto the AdvantageScope window.

### Navigating replays

- Use the **timeline** at the bottom to scrub through the match.
- All tabs (2D field, line graphs, etc.) stay synchronized.
- Click a point on the timeline to select it; right-click to deselect.
