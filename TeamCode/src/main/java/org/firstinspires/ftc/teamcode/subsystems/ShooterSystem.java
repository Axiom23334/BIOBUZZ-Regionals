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

    // Base target flywheel RPM
    public static double TARGET_RPM = 3500.0;

    // =========================================================================
    // 1x4 HORIZONTAL APRILTAG CLUSTER REGRESSION TUNING
    // Physical geometry: 4 AprilTags placed horizontally in a 1x4 row with known spacing.
    // ThriftyCam focal length (fx) = 578.272 px on 640x480 resolution.
    // Physical spacing between adjacent tag centers in FTC is typically 6.0 inches.
    // Expected pixel spacing = (PHYSICAL_TAG_SPACING_INCHES * FOCAL_LENGTH_PX) / distanceInches
    // Therefore: distanceInches = (PHYSICAL_TAG_SPACING_INCHES * FOCAL_LENGTH_PX) / tagSpacingPx
    // =========================================================================
    public static double FOCAL_LENGTH_PX = 578.272;            // ThriftyCam calibrated focal length
    public static double PHYSICAL_TAG_SPACING_INCHES = 6.0;   // Physical distance between adjacent tags in the 1x4 line
    public static double REFERENCE_TAG_SPACING_PX = 72.0;      // Nominal pixel spacing at standard shooting range (~48 in)

    // Polynomial hood angle regression coefficients based on 1x4 tag row geometry:
    // baseHoodAngle = HOOD_INTERCEPT
    //               + HOOD_COEFF_DIST * distance
    //               + HOOD_COEFF_DIST2 * distance^2
    //               + HOOD_COEFF_SPACING * (REFERENCE_TAG_SPACING_PX - perceivedSpacing)
    //               + HOOD_COEFF_BEARING * |bearing|
    //               + HOOD_COEFF_YAW * |clusterYaw|
    public static double HOOD_INTERCEPT = 0.40;          // Base offset c0
    public static double HOOD_COEFF_DIST = 0.0035;       // Distance coefficient (per inch)
    public static double HOOD_COEFF_DIST2 = 0.00001;     // Quadratic distance coefficient (per inch^2)
    public static double HOOD_COEFF_SPACING = 0.0020;    // Perceived tag spacing delta coefficient (per pixel)
    public static double HOOD_COEFF_BEARING = 0.0008;    // Cluster horizontal bearing coefficient (per degree)
    public static double HOOD_COEFF_YAW = 0.0006;        // Tag row angle / yaw perspective coefficient (per degree)
    public static double HOOD_COEFF_ELEVATION = 0.0010;  // Elevation angle coefficient (per degree)

    // Distance validity bounds (inches)
    public static double REGRESSION_MIN_DIST = 12.0;
    public static double REGRESSION_MAX_DIST = 160.0;

    // Minimum required tag spacing (pixels) to prevent division by zero / extreme noise
    public static double MIN_VALID_SPACING_PX = 10.0;

    // =========================================================================
    // FLYWHEEL VELOCITY COMPENSATION TUNING
    // When the flywheel speed drops below targetRPM (error = targetRPM - currentRPM > 0),
    // we increase the pitch angle to launch higher/counteract the loss in distance.
    // effectivePitch = basePitch + (rpmDrop * compensationGain)
    // =========================================================================
    public static double COMPENSATION_GAIN = 0.0002; // servo position delta per RPM drop
    public static double MAX_COMPENSATION_PITCH = 0.15; // maximum pitch adjustment offset
    public static double MIN_COMPENSATION_PITCH = -0.15; // lower bound if RPM overshoots
    public static double COMPENSATION_DEADBAND_RPM = 25.0; // ignore small speed fluctuations

    // Servo limits (soft limits for the physical mechanism)
    public static double MIN_SERVO_POSITION = 0.0;
    public static double MAX_SERVO_POSITION = 1.0;

    // PIDF Controller
    public static double kP = 0.0008;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.00025;
    private final PIDFController shooterPid = new PIDFController(kP, kI, kD, kF);

    // Live state tracking
    private double basePitch = 0.40;
    private double calculatedRegressionPitch = 0.40;
    private double compensatedPitch = 0.40;
    private double currentRPM = 0.0;
    private double rpmError = 0.0;
    private boolean enabled = true;
    private boolean regressionActive = false;

    // Last regression calculation details for telemetry/debugging
    private double lastEstimatedDist = 0.0;
    private double lastClusterBearing = 0.0;
    private double lastClusterYaw = 0.0;
    private double lastPerceivedSpacing = 0.0;

    public ShooterSystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        pitchServo = hardwareMap.get(Servo.class, "pitch");
    }

    /**
     * Complete 1x4 AprilTag cluster regression calculation matching telemetryAprilTag logic.
     * Takes visible detections and calculates the base hood position considering:
     * - Even if only 1, 2, or 3 tags are visible from the 4-tag row (indices 0..3)
     * - The perceived spacing between tag centers in pixels (optical distance)
     * - Solved 3D pose distance (range / ftcPose.y)
     * - Horizontal bearing from the reconstructed row center (320 - centreX)
     * - Tag yaw angle (perspective tilt of the 1x4 wall)
     *
     * @param detections Full list of AprilTagDetection objects from the processor
     * @param targetClusterStart Starting ID for the 4-tag row (e.g. 38 for audience blue, 30 for far red, etc.)
     * @param lastKnownSpacing Previous valid tag spacing fallback in pixels
     * @return Calculated base hood position
     */
    public double updateClusterRegression(List<AprilTagDetection> detections, int targetClusterStart, double lastKnownSpacing) {
        if (detections == null || detections.isEmpty()) {
            regressionActive = false;
            return basePitch;
        }

        double sumX = 0;
        double sumY = 0;
        double sumIndex = 0;
        int count = 0;

        // Collect poses and yaw angles
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
            return basePitch;
        }

        // Calculate spacing between visible tags in the 1x4 line
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

        // Calculate the reconstructed row geometric center (handling partial visibility 1..3 tags)
        double averageIndex = sumIndex / count;
        double centreX = (sumX / count) + (1.5 - averageIndex) * perceivedSpacing;
        double centreY = sumY / count;

        // Cluster bearing angle in degrees relative to camera center line (640 wide, center = 320)
        // Horizontal field of view ~ 60 degrees -> 320 px = 30 deg -> ~0.09375 deg/px
        double clusterBearing = (centreX - 320.0) * (60.0 / 640.0);

        // Determine best distance measurement:
        // Prioritize fused optical triangulation from 1x4 tag spacing: d = (physical_spacing * f) / pixel_spacing
        double opticalDistance = (PHYSICAL_TAG_SPACING_INCHES * FOCAL_LENGTH_PX) / perceivedSpacing;
        double solvedDistance = (poseCount > 0) ? (sumRange / poseCount) : opticalDistance;

        // Blend 3D pose distance and optical spacing distance for maximum reliability
        double distance = (poseCount > 0) ? (0.6 * solvedDistance + 0.4 * opticalDistance) : opticalDistance;
        distance = Range.clip(distance, REGRESSION_MIN_DIST, REGRESSION_MAX_DIST);

        double clusterYaw = (poseCount > 0) ? (sumYaw / poseCount) : 0.0;
        double clusterElevation = (poseCount > 0) ? (sumElevation / poseCount) : 0.0;

        // Apply regression formula
        double distTerm = (HOOD_COEFF_DIST * distance) + (HOOD_COEFF_DIST2 * distance * distance);
        double spacingTerm = HOOD_COEFF_SPACING * (REFERENCE_TAG_SPACING_PX - perceivedSpacing);
        double angleTerm = (HOOD_COEFF_BEARING * Math.abs(clusterBearing))
                + (HOOD_COEFF_YAW * Math.abs(clusterYaw))
                + (HOOD_COEFF_ELEVATION * clusterElevation);

        calculatedRegressionPitch = Range.clip(
                HOOD_INTERCEPT + distTerm + spacingTerm + angleTerm,
                MIN_SERVO_POSITION,
                MAX_SERVO_POSITION
        );

        this.basePitch = calculatedRegressionPitch;
        this.regressionActive = true;

        // Cache parameters for telemetry
        this.lastEstimatedDist = distance;
        this.lastClusterBearing = clusterBearing;
        this.lastClusterYaw = clusterYaw;
        this.lastPerceivedSpacing = perceivedSpacing;

        return this.basePitch;
    }

    /**
     * Direct regression evaluation using distance, bearing, elevation, and spacing.
     */
    public double updateRegression(double distanceInches, double bearingDegrees, double elevationDegrees, double perceivedSpacingPx) {
        if (distanceInches <= 0 || Double.isNaN(distanceInches)) {
            regressionActive = false;
            return basePitch;
        }

        double clampedDist = Range.clip(distanceInches, REGRESSION_MIN_DIST, REGRESSION_MAX_DIST);

        double distTerm = (HOOD_COEFF_DIST * clampedDist) + (HOOD_COEFF_DIST2 * clampedDist * clampedDist);
        double spacingTerm = 0.0;
        if (perceivedSpacingPx > 0) {
            spacingTerm = HOOD_COEFF_SPACING * (REFERENCE_TAG_SPACING_PX - perceivedSpacingPx);
        }
        double angleTerm = (HOOD_COEFF_BEARING * Math.abs(bearingDegrees)) + (HOOD_COEFF_ELEVATION * elevationDegrees);

        calculatedRegressionPitch = Range.clip(
                HOOD_INTERCEPT + distTerm + spacingTerm + angleTerm,
                MIN_SERVO_POSITION,
                MAX_SERVO_POSITION
        );

        this.basePitch = calculatedRegressionPitch;
        this.regressionActive = true;
        this.lastEstimatedDist = clampedDist;
        this.lastClusterBearing = bearingDegrees;
        this.lastPerceivedSpacing = perceivedSpacingPx;

        return this.basePitch;
    }

    /**
     * Updates the flywheel PID loop and applies velocity-compensated hood/pitch position.
     * Call this in every loop iteration.
     */
    public void update() {
        shooterPid.setPIDF(kP, kI, kD, kF);

        // Read current flywheel RPM (negated due to motor orientation)
        double currentTicksPerSec = shooterMotor.getVelocity();
        currentRPM = (-currentTicksPerSec * 60.0) / SHOOTER_TICKS_PER_REV;

        if (enabled) {
            shooterPid.setSetPoint(TARGET_RPM);
            double shooterPower = shooterPid.calculate(currentRPM);
            shooterMotor.setPower(Range.clip(shooterPower, -1.0, 1.0));

            // Velocity compensation calculation
            rpmError = TARGET_RPM - currentRPM;
            double pitchAdjustment = 0.0;

            if (Math.abs(rpmError) > COMPENSATION_DEADBAND_RPM) {
                pitchAdjustment = Range.clip(
                        rpmError * COMPENSATION_GAIN,
                        MIN_COMPENSATION_PITCH,
                        MAX_COMPENSATION_PITCH
                );
            }

            compensatedPitch = Range.clip(basePitch + pitchAdjustment, MIN_SERVO_POSITION, MAX_SERVO_POSITION);
        } else {
            shooterMotor.setPower(0);
            compensatedPitch = Range.clip(basePitch, MIN_SERVO_POSITION, MAX_SERVO_POSITION);
        }

        pitchServo.setPosition(compensatedPitch);
    }

    public void setBasePitch(double angle) {
        this.basePitch = Range.clip(angle, MIN_SERVO_POSITION, MAX_SERVO_POSITION);
        this.regressionActive = false;
    }

    public void adjustBasePitch(double delta) {
        this.basePitch = Range.clip(this.basePitch + delta, MIN_SERVO_POSITION, MAX_SERVO_POSITION);
        this.regressionActive = false;
    }

    public double getBasePitch() {
        return basePitch;
    }

    public double getCalculatedRegressionPitch() {
        return calculatedRegressionPitch;
    }

    public double getCompensatedPitch() {
        return compensatedPitch;
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
