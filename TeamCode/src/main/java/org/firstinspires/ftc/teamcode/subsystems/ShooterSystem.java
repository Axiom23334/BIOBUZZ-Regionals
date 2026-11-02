package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
public class ShooterSystem {
    // Hardware references
    private final DcMotorEx shooterMotor;
    private final Servo pitchServo;

    // Motor encoder specs
    public static double SHOOTER_TICKS_PER_REV = 28.0;

    // Base target flywheel RPM (Default baseline)
    public static double TARGET_RPM = 3500.0;

    // =========================================================================
    // 2-DIMENSIONAL REGRESSION (HOOD PITCH & FLYWHEEL RPM)
    // Both variables are calculated directly based on target distance d:
    // 1. targetRPM = RPM_INTERCEPT + RPM_COEFF_DIST * d + RPM_COEFF_DIST2 * d^2
    // 2. targetPitch = HOOD_INTERCEPT + HOOD_COEFF_DIST * d + HOOD_COEFF_DIST2 * d^2
    //                  + HOOD_COEFF_BEARING * |bearing| + HOOD_COEFF_YAW * |yaw|
    // =========================================================================

    // Optical geometry constants for ThriftyCam (640x480)
    public static double FOCAL_LENGTH_PX = 578.272;            // ThriftyCam calibrated focal length
    public static double PHYSICAL_TAG_SPACING_INCHES = 6.0;   // Physical spacing between adjacent tag centers in 1x4 line
    public static double REFERENCE_TAG_SPACING_PX = 72.0;      // Nominal pixel spacing at ~48 inches

    // --- Dimension 1: Hood Pitch Regression Tuning ---
    public static double HOOD_INTERCEPT = 0.38;          // Base hood offset
    public static double HOOD_COEFF_DIST = 0.0035;       // Linear distance coefficient (per inch)
    public static double HOOD_COEFF_DIST2 = 0.00001;     // Quadratic distance coefficient (per inch^2)
    public static double HOOD_COEFF_SPACING = 0.0018;    // Tag spacing delta coefficient (per pixel)
    public static double HOOD_COEFF_BEARING = 0.0006;    // Bearing angle coefficient (per degree)
    public static double HOOD_COEFF_YAW = 0.0005;        // Perspective yaw angle coefficient (per degree)

    // --- Dimension 2: Flywheel RPM Regression Tuning ---
    public static double RPM_INTERCEPT = 3200.0;         // Base flywheel RPM at point-blank range
    public static double RPM_COEFF_DIST = 5.5;           // Additional RPM per inch of distance
    public static double RPM_COEFF_DIST2 = 0.015;        // Quadratic RPM coefficient (per inch^2)
    public static double RPM_MIN = 2800.0;               // RPM safety lower bound
    public static double RPM_MAX = 4200.0;               // RPM safety upper bound

    // Distance validity bounds (inches)
    public static double REGRESSION_MIN_DIST = 12.0;
    public static double REGRESSION_MAX_DIST = 160.0;
    public static double MIN_VALID_SPACING_PX = 10.0;

    // Servo limits (soft limits for the physical mechanism)
    public static double MIN_SERVO_POSITION = 0.0;
    public static double MAX_SERVO_POSITION = 1.0;

    // Flywheel PIDF Controller
    public static double kP = 0.0008;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.00025;
    private final PIDFController shooterPid = new PIDFController(kP, kI, kD, kF);

    // Live state tracking
    private double currentPitch = 0.40;
    private double calculatedRegressionPitch = 0.40;
    private double calculatedRegressionRPM = 3500.0;
    private double activeTargetRPM = 3500.0;
    private double currentRPM = 0.0;
    private double rpmError = 0.0;
    private boolean enabled = true;
    private boolean regressionActive = false;

    // Centerpoint and geometry tracking for diagnostics
    private double lastEstimatedDist = 0.0;
    private double lastClusterBearing = 0.0;
    private double lastClusterYaw = 0.0;
    private double lastPerceivedSpacing = 0.0;
    private double lastCentreX = 320.0;
    private double lastCentreY = 240.0;

    public ShooterSystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        pitchServo = hardwareMap.get(Servo.class, "pitch");
    }

    /**
     * Solves the true centrepoint of the 1x4 AprilTag cluster (even with partial 1..3 tag visibility)
     * and calculates the 2-dimensional regression for:
     * 1. Optimal Hood Pitch
     * 2. Optimal Flywheel RPM
     *
     * @param detections Full list of AprilTagDetection objects from the processor
     * @param targetClusterStart Starting ID for the 4-tag row (e.g. 38 for audience blue, 30 for far red)
     * @param lastKnownSpacing Previous valid tag spacing fallback in pixels
     * @return Calculated hood pitch
     */
    public double updateClusterRegression(List<AprilTagDetection> detections, int targetClusterStart, double lastKnownSpacing) {
        if (detections == null || detections.isEmpty()) {
            regressionActive = false;
            return currentPitch;
        }

        // =====================================================================
        // ROBUST 1x4 CLUSTER CENTREPOINT CALCULATION
        // Even if only a subset (e.g. tags 0 and 2, or just tag 1) are visible,
        // we can mathematically locate the exact midpoint of all 4 tags (index 1.5).
        // =====================================================================
        double sumX = 0;
        double sumY = 0;
        double sumIndex = 0;
        int count = 0;

        double sumRange = 0;
        double sumYaw = 0;
        double sumElevation = 0;
        int poseCount = 0;

        for (AprilTagDetection detection : detections) {
            int index = detection.id - targetClusterStart;
            if (index >= 0 && index <= 3) {
                sumX += detection.center.x;
                sumY += detection.center.y;
                sumIndex += index;
                count++;

                if (detection.ftcPose != null) {
                    sumRange += detection.ftcPose.range;
                    sumYaw += detection.ftcPose.yaw;
                    sumElevation += detection.ftcPose.elevation;
                    poseCount++;
                }
            }
        }

        if (count == 0) {
            regressionActive = false;
            return currentPitch;
        }

        // Pairwise inter-tag spacing calculation normalized by index difference
        double sumSpacing = 0;
        int pairs = 0;

        for (AprilTagDetection a : detections) {
            int indexA = a.id - targetClusterStart;
            if (indexA < 0 || indexA > 3) continue;

            for (AprilTagDetection b : detections) {
                int indexB = b.id - targetClusterStart;
                if (indexB <= indexA || indexB > 3) continue;

                sumSpacing += (b.center.x - a.center.x) / (indexB - indexA);
                pairs++;
            }
        }

        double perceivedSpacing = (pairs > 0) ? (sumSpacing / pairs) : lastKnownSpacing;
        if (perceivedSpacing < MIN_VALID_SPACING_PX) {
            perceivedSpacing = (lastKnownSpacing >= MIN_VALID_SPACING_PX) ? lastKnownSpacing : REFERENCE_TAG_SPACING_PX;
        }

        // Centre of the complete four-tag row (index 1.5):
        // centreX = (centroidX) + (1.5 - centroidIndex) * spacing
        double averageIndex = sumIndex / count;
        lastCentreX = (sumX / count) + (1.5 - averageIndex) * perceivedSpacing;
        lastCentreY = sumY / count;

        // Bearing angle from camera centerline (640px wide, center = 320px)
        lastClusterBearing = (lastCentreX - 320.0) * (60.0 / 640.0);

        // Distance Triangulation:
        // Optical distance from known physical tag spacing (6 inches)
        double opticalDistance = (PHYSICAL_TAG_SPACING_INCHES * FOCAL_LENGTH_PX) / perceivedSpacing;
        double solvedDistance = (poseCount > 0) ? (sumRange / poseCount) : opticalDistance;

        // Fused distance estimate
        double distance = (poseCount > 0) ? (0.6 * solvedDistance + 0.4 * opticalDistance) : opticalDistance;
        distance = Range.clip(distance, REGRESSION_MIN_DIST, REGRESSION_MAX_DIST);

        lastClusterYaw = (poseCount > 0) ? (sumYaw / poseCount) : 0.0;
        double clusterElevation = (poseCount > 0) ? (sumElevation / poseCount) : 0.0;

        // =====================================================================
        // 2-DIMENSIONAL REGRESSION EVALUATION
        // =====================================================================
        // Dimension 1: Hood Pitch
        double distPitchTerm = (HOOD_COEFF_DIST * distance) + (HOOD_COEFF_DIST2 * distance * distance);
        double spacingPitchTerm = HOOD_COEFF_SPACING * (REFERENCE_TAG_SPACING_PX - perceivedSpacing);
        double anglePitchTerm = (HOOD_COEFF_BEARING * Math.abs(lastClusterBearing))
                + (HOOD_COEFF_YAW * Math.abs(lastClusterYaw))
                + (0.0010 * clusterElevation);

        calculatedRegressionPitch = Range.clip(
                HOOD_INTERCEPT + distPitchTerm + spacingPitchTerm + anglePitchTerm,
                MIN_SERVO_POSITION,
                MAX_SERVO_POSITION
        );

        // Dimension 2: Flywheel Target RPM
        double distRpmTerm = (RPM_COEFF_DIST * distance) + (RPM_COEFF_DIST2 * distance * distance);
        calculatedRegressionRPM = Range.clip(RPM_INTERCEPT + distRpmTerm, RPM_MIN, RPM_MAX);

        this.currentPitch = calculatedRegressionPitch;
        this.activeTargetRPM = calculatedRegressionRPM;
        this.regressionActive = true;

        this.lastEstimatedDist = distance;
        this.lastPerceivedSpacing = perceivedSpacing;

        return this.currentPitch;
    }

    /**
     * Updates the flywheel PID loop and applies the hood pitch position.
     * Call this in every loop iteration.
     */
    public void update() {
        shooterPid.setPIDF(kP, kI, kD, kF);

        // Read current flywheel RPM (negated due to motor orientation)
        double currentTicksPerSec = shooterMotor.getVelocity();
        currentRPM = (-currentTicksPerSec * 60.0) / SHOOTER_TICKS_PER_REV;

        double targetSpeed = regressionActive ? activeTargetRPM : TARGET_RPM;

        if (enabled) {
            shooterPid.setSetPoint(targetSpeed);
            double shooterPower = shooterPid.calculate(currentRPM);
            shooterMotor.setPower(Range.clip(shooterPower, -1.0, 1.0));
            rpmError = targetSpeed - currentRPM;
        } else {
            shooterMotor.setPower(0);
            rpmError = 0.0;
        }

        pitchServo.setPosition(Range.clip(currentPitch, MIN_SERVO_POSITION, MAX_SERVO_POSITION));
    }

    public void setBasePitch(double angle) {
        this.currentPitch = Range.clip(angle, MIN_SERVO_POSITION, MAX_SERVO_POSITION);
        this.regressionActive = false;
    }

    public void adjustBasePitch(double delta) {
        this.currentPitch = Range.clip(this.currentPitch + delta, MIN_SERVO_POSITION, MAX_SERVO_POSITION);
        this.regressionActive = false;
    }

    public void setTargetRPM(double rpm) {
        this.activeTargetRPM = Range.clip(rpm, RPM_MIN, RPM_MAX);
        TARGET_RPM = this.activeTargetRPM;
        this.regressionActive = false;
    }

    public void adjustTargetRPM(double deltaRPM) {
        setTargetRPM(this.activeTargetRPM + deltaRPM);
    }

    public double getBasePitch() {
        return currentPitch;
    }

    public double getPitch() {
        return currentPitch;
    }

    public double getCompensatedPitch() {
        return currentPitch; // Alias for backward compatibility
    }

    public double getCalculatedRegressionPitch() {
        return calculatedRegressionPitch;
    }

    public double getCalculatedRegressionRPM() {
        return calculatedRegressionRPM;
    }

    public double getActiveTargetRPM() {
        return regressionActive ? activeTargetRPM : TARGET_RPM;
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    public double getRpmError() {
        return rpmError;
    }

    public boolean isRegressionActive() {
        return regressionActive;
    }

    public double getLastEstimatedDist() {
        return lastEstimatedDist;
    }

    public double getLastClusterBearing() {
        return lastClusterBearing;
    }

    public double getLastClusterYaw() {
        return lastClusterYaw;
    }

    public double getLastPerceivedSpacing() {
        return lastPerceivedSpacing;
    }

    public double getLastCentreX() {
        return lastCentreX;
    }

    public double getLastCentreY() {
        return lastCentreY;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void stop() {
        enabled = false;
        shooterMotor.setPower(0);
    }

    public DcMotorEx getShooterMotor() {
        return shooterMotor;
    }

    public Servo getPitchServo() {
        return pitchServo;
    }
}
