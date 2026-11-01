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
    private GoBildaPinpointDriver pin;

    // Turret encoder & physical limits
    // Soft limits in encoder ticks relative to 0 (straight forward on robot).
    // 450.0 ticks @ 2.5 ticks/deg = 180 degrees in either direction (+-180 deg)
    public static double TICKS_PER_DEGREE = 2.5;
    public static double TURRET_LIMIT_TICKS = 450.0; // 180 degrees left and right
    public static double DEAD_BAND_PIXELS = 2.5;     // Pixel error deadband

    // Vision Control Gains (Primary Driver)
    // Note: pixelErrorX = 320 - centreX. positive pixelErrorX means target is LEFT.
    // Negative motor power rotates turret LEFT on this robot hardware.
    public static double KP_VISION = -0.0018;   // Proportional vision gain (negative)
    public static double KD_VISION = 0.00015;   // Velocity damping gain (positive)
    public static double MIN_MOVE_POWER = 0.05; // Overcome static motor friction
    public static double MAX_POWER = 0.85;      // Maximum motor power clamp

    // Pinpoint Position Holding Gains (When vision drops)
    public static double KP_HOLD = 0.0025;      // Proportional position holding gain in ticks (positive)
    public static double KD_HOLD = 0.0001;      // Velocity damping gain
    public static double KP_FEEDFORWARD_GYRO = 1.0; // Gyro counter-rotation feedforward

    // Filter settings
    public static double FILTER_ALPHA = 0.40;   // Low-pass filter weight on pixel error (0..1)
    public static double MAX_ALLOWED_PIXEL_JUMP = 100.0; // Outlier jump rejection (pixels)

    // Target memory & state tracking
    private boolean hasTarget = false;
    private boolean pinpointTracking = false;
    private double filteredPixelError = 0.0;

    private double targetTurretHeading = 0.0; // Saved world heading of AprilTag target in degrees
    private double currentBotHeading = 0.0;   // Current robot heading from Pinpoint in degrees
    private double lastBotHeading = Double.NaN;
    private double botYawRateDegSec = 0.0;

    // State & Timers
    private String trackerState = "IDLE";     // "VISION", "PINPOINT_HOLD", "IDLE"
    private String lastCell = "NONE";         // "AUDIENCE" or "FAR"
    private final ElapsedTime visionLossTimer = new ElapsedTime();
    private final ElapsedTime dtTimer = new ElapsedTime();
    public static double VISION_TIMEOUT_SECONDS = 4.0; // Retain target lock up to 4 seconds after vision loss

    public TurretTracker(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtTimer.reset();
        visionLossTimer.reset();
    }

    public void setPin(GoBildaPinpointDriver pin) {
        this.pin = pin;
    }

    public void setOdo(GoBildaPinpointDriver pin) {
        setPin(pin);
    }

    /**
     * Reads robot heading from Pinpoint internal IMU if valid.
     * Uses ONLY_UPDATE_HEADING to avoid pod fault stalls or delays.
     */
    public double updateHeadingFromPinpoint() {
        if (pin == null) return Double.NaN;
        try {
            pin.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
            double rawRad = pin.getHeading(AngleUnit.RADIANS);
            double rawDeg = Math.toDegrees(rawRad);
            if (Double.isNaN(rawDeg)) return Double.NaN;
            currentBotHeading = AngleUnit.normalizeDegrees(rawDeg * SwerveTeleOpConfig.IMU_POLARITY);
            return currentBotHeading;
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    /**
     * Main tracking update.
     *
     * @param pixelErrorX 320 - centreX (from vision). Pass -99999 when no AprilTag is detected.
     * @param cell "AUDIENCE", "FAR", or "NONE"
     * @return Motor power applied to turret
     */
    public double update(double pixelErrorX, String cell) {
        double botHeading = updateHeadingFromPinpoint();
        boolean pinpointAvailable = !Double.isNaN(botHeading);
        double currentTicks = turretMotor.getCurrentPosition();
        double currentVelocity = turretMotor.getVelocity(); // ticks/second

        double dt = dtTimer.seconds();
        if (dt <= 0.001) dt = 0.001;
        dtTimer.reset();

        // Calculate chassis yaw rate (deg/s) from Pinpoint
        if (pinpointAvailable) {
            if (!Double.isNaN(lastBotHeading)) {
                double deltaAngle = AngleUnit.normalizeDegrees(botHeading - lastBotHeading);
                botYawRateDegSec = deltaAngle / dt;
            }
            lastBotHeading = botHeading;
        } else {
            botYawRateDegSec = 0.0;
        }

        double power = 0.0;

        if (pixelErrorX != -99999) {
            // =================================================================
            // CASE 1: VISION IS ACTIVE
            // =================================================================
            if (hasTarget && Math.abs(pixelErrorX - filteredPixelError) > MAX_ALLOWED_PIXEL_JUMP && Math.abs(filteredPixelError) > 0.1) {
                pixelErrorX = filteredPixelError + Math.signum(pixelErrorX - filteredPixelError) * MAX_ALLOWED_PIXEL_JUMP;
            }

            if (!hasTarget) {
                filteredPixelError = pixelErrorX;
            } else {
                filteredPixelError = (FILTER_ALPHA * filteredPixelError) + ((1.0 - FILTER_ALPHA) * pixelErrorX);
            }

            hasTarget = true;
            lastCell = cell;
            visionLossTimer.reset();
            trackerState = "VISION";

            // Vision PD Control:
            // pixelErrorX > 0 (tag left) -> power < 0 (rotate left)
            // KD_VISION opposes motor velocity currentVelocity
            if (Math.abs(filteredPixelError) > DEAD_BAND_PIXELS) {
                power = (filteredPixelError * KP_VISION) - (currentVelocity * KD_VISION);
                if (Math.abs(power) > 0.01 && Math.abs(power) < MIN_MOVE_POWER) {
                    power = Math.signum(power) * MIN_MOVE_POWER;
                }
            } else {
                power = 0.0;
            }

            // Continuously memorize the target's world heading via Pinpoint
            if (pinpointAvailable) {
                double turretRelAngleDeg = currentTicks / TICKS_PER_DEGREE;
                double visionAngleOffsetDeg = (-filteredPixelError * (60.0 / 640.0));
                double calculatedWorldHeading = AngleUnit.normalizeDegrees(botHeading + turretRelAngleDeg + visionAngleOffsetDeg);

                // Low pass filter saved target world heading to eliminate vision jitter
                if (targetTurretHeading == 0.0) {
                    targetTurretHeading = calculatedWorldHeading;
                } else {
                    double headingDiff = AngleUnit.normalizeDegrees(calculatedWorldHeading - targetTurretHeading);
                    targetTurretHeading = AngleUnit.normalizeDegrees(targetTurretHeading + 0.3 * headingDiff);
                }
                pinpointTracking = true;
            }

            // Gyro feedforward against robot chassis spin
            if (pinpointAvailable && Math.abs(botYawRateDegSec) > 1.0) {
                double gyroFeedforward = -botYawRateDegSec * (TICKS_PER_DEGREE / 1000.0) * KP_FEEDFORWARD_GYRO;
                power += gyroFeedforward;
            }

        } else if (hasTarget && pinpointAvailable && visionLossTimer.seconds() < VISION_TIMEOUT_SECONDS) {
            // =================================================================
            // CASE 2: VISION LOST - HOLD FIELD TARGET POSITION USING PINPOINT
            // =================================================================
            trackerState = "PINPOINT_HOLD";

            double desiredTurretAngleDeg = AngleUnit.normalizeDegrees(targetTurretHeading - botHeading);
            double desiredTicks = desiredTurretAngleDeg * TICKS_PER_DEGREE;
            desiredTicks = Range.clip(desiredTicks, -TURRET_LIMIT_TICKS, TURRET_LIMIT_TICKS);

            double tickError = desiredTicks - currentTicks;
            power = (tickError * KP_HOLD) - (currentVelocity * KD_HOLD);

            // Gyro feedforward against active chassis rotation
            double gyroFeedforward = -botYawRateDegSec * (TICKS_PER_DEGREE / 1000.0) * KP_FEEDFORWARD_GYRO;
            power += gyroFeedforward;

        } else {
            // =================================================================
            // CASE 3: NO TARGET OR TIMEOUT EXCEEDED
            // =================================================================
            hasTarget = false;
            pinpointTracking = false;
            filteredPixelError = 0.0;
            targetTurretHeading = 0.0;
            power = 0.0;
            trackerState = "IDLE";
        }

        // Apply physical soft limit protection [-TURRET_LIMIT_TICKS, TURRET_LIMIT_TICKS] (+-180 degrees)
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
        hasTarget = false;
        targetTurretHeading = 0.0;
        trackerState = "STOPPED";
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

    public double getFilteredPixelError() {
        return filteredPixelError;
    }

    public double getBotYawRateDegSec() {
        return botYawRateDegSec;
    }

    public String getTrackerState() {
        return trackerState;
    }

    public String getLastCell() {
        return lastCell;
    }

    public DcMotorEx getTurretMotor() {
        return turretMotor;
    }
}
