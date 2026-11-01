package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.SwerveTeleOpConfig;
import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;

@Config
public class TurretTracker {
    private final DcMotorEx turretMotor;
    private GoBildaPinpointDriver odo;

    // Turret encoder & physical limits
    // Soft limits in encoder ticks relative to 0 (straight forward on robot)
    public static double TURRET_LIMIT_TICKS = 300.0;
    public static double TICKS_PER_DEGREE = 2.5; // Conversion between turret encoder ticks and degrees
    public static double DEAD_BAND_PIXELS = 4.0; // Pixel error deadband

    // Control gains (Dashboard Configurable)
    public static double KP_VISION = -0.0012;   // Proportional gain for direct camera pixel error (320 - centreX)
    public static double KP_HEADING = 0.0035;   // Heading feedforward gain to counteract robot yaw
    public static double KP_HOLD = 0.0020;      // Proportional position holding gain in ticks
    public static double MAX_POWER = 0.75;      // Clamp to prevent aggressive oscillation or cable wrap

    // Target memory & state tracking
    private boolean hasTarget = false;
    private boolean pinpointTracking = false;
    private double lastTargetCentreX = 320.0;
    private double lastTargetCentreY = 240.0;
    private double targetTurretHeading = 0.0; // Estimated world/field heading of the AprilTag cluster in degrees
    private double currentBotHeading = 0.0;   // Current robot heading from Pinpoint in degrees
    private double targetTicks = 0.0;         // Desired turret encoder tick setpoint
    private String lastCell = "NONE";         // "AUDIENCE" or "FAR"
    private final ElapsedTime visionLossTimer = new ElapsedTime();
    public static double VISION_TIMEOUT_SECONDS = 3.0; // Retain tracking memory up to 3 seconds after loss

    public TurretTracker(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setOdo(GoBildaPinpointDriver odo) {
        this.odo = odo;
    }

    /**
     * Reads robot heading from Pinpoint internal IMU if valid.
     * Works with or without external odometry deadwheels plugged in.
     * Uses ONLY_UPDATE_HEADING to avoid pod fault stalls or delays.
     *
     * @return current robot heading in degrees (normalized -180..180), or NaN if unavailable.
     */
    public double updateHeadingFromPinpoint() {
        if (odo == null) return Double.NaN;
        try {
            // Specifically read IMU heading directly, bypassing missing pod check
            odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
            double rawRad = odo.getHeading(AngleUnit.RADIANS);
            double rawDeg = Math.toDegrees(rawRad);
            if (Double.isNaN(rawDeg)) return Double.NaN;
            currentBotHeading = AngleUnit.normalizeDegrees(rawDeg * SwerveTeleOpConfig.IMU_POLARITY);
            return currentBotHeading;
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    /**
     * Main tracking update. Call every loop cycle.
     *
     * @param pixelErrorX 320 - centreX (from vision). Pass -99999 when no AprilTag is detected.
     * @param cell "AUDIENCE", "FAR", or "NONE"
     * @return Motor power applied to turret
     */
    public double update(double pixelErrorX, String cell) {
        double botHeading = updateHeadingFromPinpoint();
        boolean pinpointAvailable = !Double.isNaN(botHeading);
        double currentTicks = turretMotor.getCurrentPosition();
        double power = 0.0;

        if (pixelErrorX != -99999) {
            // =================================================================
            // CASE 1: VISION IS ACTIVE (AprilTag cluster visible)
            // =================================================================
            hasTarget = true;
            lastTargetCentreX = 320.0 - pixelErrorX;
            lastCell = cell;
            visionLossTimer.reset();

            // Direct proportional vision control
            if (Math.abs(pixelErrorX) > DEAD_BAND_PIXELS) {
                power = pixelErrorX * KP_VISION;
            } else {
                power = 0.0;
            }

            // If Pinpoint is available, calculate and memorize the target's world heading:
            // Turret angle (deg) relative to robot front ≈ currentTicks / TICKS_PER_DEGREE
            // Target angle relative to turret ≈ pixelErrorX * (horizontalFOV / 640)
            if (pinpointAvailable) {
                double turretRelAngleDeg = currentTicks / TICKS_PER_DEGREE;
                double visionAngleOffsetDeg = -pixelErrorX * (60.0 / 640.0);
                targetTurretHeading = AngleUnit.normalizeDegrees(botHeading + turretRelAngleDeg + visionAngleOffsetDeg);
                targetTicks = currentTicks;
                pinpointTracking = true;
            }
        } else if (hasTarget && pinpointAvailable && visionLossTimer.seconds() < VISION_TIMEOUT_SECONDS) {
            // =================================================================
            // CASE 2: VISION DROPPED, BUT PINPOINT HEADING IS AVAILABLE
            // Maintain orientation locked onto the memorized target position in world space!
            // =================================================================
            pinpointTracking = true;
            // Needed turret angle relative to robot = targetWorldHeading - currentRobotHeading
            double desiredTurretAngleDeg = AngleUnit.normalizeDegrees(targetTurretHeading - botHeading);
            double desiredTicks = desiredTurretAngleDeg * TICKS_PER_DEGREE;

            // Clamped position error
            double tickError = desiredTicks - currentTicks;
            power = tickError * KP_HOLD;
        } else {
            // =================================================================
            // CASE 3: NO TARGET OR TIMED OUT
            // =================================================================
            hasTarget = false;
            pinpointTracking = false;
            power = 0.0;
        }

        // Apply soft limit protection [-TURRET_LIMIT_TICKS, TURRET_LIMIT_TICKS]
        if (currentTicks >= TURRET_LIMIT_TICKS && power > 0) {
            power = 0.0;
        } else if (currentTicks <= -TURRET_LIMIT_TICKS && power < 0) {
            power = 0.0;
        }

        power = Range.clip(power, -MAX_POWER, MAX_POWER);
        turretMotor.setPower(power);
        return power;
    }

    public void stop() {
        turretMotor.setPower(0.0);
    }

    public boolean isTracking() {
        return hasTarget;
    }

    public boolean isPinpointAssisted() {
        return pinpointTracking;
    }

    public double getTargetTurretHeading() {
        return targetTurretHeading;
    }

    public double getTurretPosition() {
        return turretMotor.getCurrentPosition();
    }

    public String getLastCell() {
        return lastCell;
    }

    public DcMotorEx getTurretMotor() {
        return turretMotor;
    }
}
