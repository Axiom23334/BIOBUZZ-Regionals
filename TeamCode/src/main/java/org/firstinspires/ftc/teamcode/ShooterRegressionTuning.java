package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;
import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp(name = "Tuning: Shooter 2D Regression", group = "Tuning")
public class ShooterRegressionTuning extends LinearOpMode {

    // Subsystems
    private ShooterSystem shooterSystem;
    private TurretTracker turretTracker;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;

    // Vision
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Alliance & tracking fallback
    public static boolean BLUE_ALLIANCE = true;
    private double lastTagSpacing = 40.0;
    private String lastDetectedCell = "NONE";

    // Tuning Step Sizes
    public static double PITCH_STEP_FINE = 0.001;
    public static double PITCH_STEP_COARSE = 0.01;
    public static double RPM_STEP_FINE = 25.0;
    public static double RPM_STEP_COARSE = 100.0;
    public static double TURRET_OFFSET_STEP_PX = 1.0;

    // Edge detectors for buttons
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastBumperLeft = false;
    private boolean lastBumperRight = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Hardware
        shooterSystem = new ShooterSystem(hardwareMap);
        turretTracker = new TurretTracker(hardwareMap);
        try {
            GoBildaPinpointDriver pin = hardwareMap.get(GoBildaPinpointDriver.class, "pin");
            turretTracker.setPin(pin);
        } catch (Exception ignored) {}

        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
            transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception ignored) {}

        initAprilTag();

        telemetry.addLine("=== SHOOTER 2D REGRESSION TUNER ===");
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("  Y / A               : Adjust Base Pitch (+/- 0.001 fine, or hold bumper for 0.01)");
        telemetry.addLine("  D-pad UP / DOWN     : Adjust Target RPM (+/- 25 fine, or hold bumper for 100)");
        telemetry.addLine("  D-pad LEFT / RIGHT  : Adjust Turret Pixel Offset (+/- 1 px)");
        telemetry.addLine("  Right Trigger       : Feed and Shoot (Runs Intake & Transfer)");
        telemetry.addLine("  Left Trigger        : Stop Flywheel");
        telemetry.addLine("  Button X            : Re-enable Auto 2D Regression");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            boolean coarseMode = gamepad1.left_bumper || gamepad1.right_bumper;

            // 1. Manual Hood Pitch Adjustment
            if (gamepad1.y) {
                shooterSystem.adjustBasePitch(coarseMode ? PITCH_STEP_COARSE : PITCH_STEP_FINE);
            } else if (gamepad1.a) {
                shooterSystem.adjustBasePitch(coarseMode ? -PITCH_STEP_COARSE : -PITCH_STEP_FINE);
            }

            // 2. Manual RPM Adjustment (D-pad Up / Down edge triggered)
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            if (dpadUp && !lastDpadUp) {
                shooterSystem.adjustTargetRPM(coarseMode ? RPM_STEP_COARSE : RPM_STEP_FINE);
            } else if (dpadDown && !lastDpadDown) {
                shooterSystem.adjustTargetRPM(coarseMode ? -RPM_STEP_COARSE : -RPM_STEP_FINE);
            }
            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;

            // 3. Manual Turret Offset Adjustment (D-pad Left / Right edge triggered)
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;
            if (dpadLeft && !lastDpadLeft) {
                turretTracker.adjustAimOffsetPixels(TURRET_OFFSET_STEP_PX); // Shift aim left
            } else if (dpadRight && !lastDpadRight) {
                turretTracker.adjustAimOffsetPixels(-TURRET_OFFSET_STEP_PX); // Shift aim right
            }
            lastDpadLeft = dpadLeft;
            lastDpadRight = dpadRight;

            // 4. Vision & 1x4 Cluster Processing
            double pixelErrorX = updateVisionAndRegression();

            // 5. Update Turret Tracker with aim offset
            double turretPower = turretTracker.update(pixelErrorX, lastDetectedCell);

            // 6. Flywheel Control & Feeder Trigger
            shooterSystem.setEnabled(!gamepad1.left_trigger_pressed);
            shooterSystem.update();

            // Right trigger feeds balls into shooter
            if (gamepad1.right_trigger > 0.3) {
                if (intakeMotor != null) intakeMotor.setPower(1.0);
                if (transferMotor != null) transferMotor.setPower(1.0);
            } else {
                if (intakeMotor != null) intakeMotor.setPower(0.0);
                if (transferMotor != null) transferMotor.setPower(0.0);
            }

            // 7. Telemetry Output for Logging & Graphing
            telemetry.addLine("===== TUNING VALUES (RECORD THESE) =====");
            telemetry.addData("Est. Target Distance", "%.1f in", shooterSystem.getLastEstimatedDist());
            telemetry.addData("Live Hood Pitch", "%.3f (Compensated: %.3f)", shooterSystem.getBasePitch(), shooterSystem.getCompensatedPitch());
            telemetry.addData("Target Flywheel RPM", "%.0f (Current: %.0f)", shooterSystem.getActiveTargetRPM(), shooterSystem.getCurrentRPM());
            telemetry.addData("RPM Error", "%.0f RPM", shooterSystem.getRpmError());

            telemetry.addLine("\n===== 2D REGRESSION SOLVER =====");
            telemetry.addData("Regression Mode", shooterSystem.isRegressionActive() ? "ACTIVE (Auto Solved)" : "MANUAL OVERRIDE");
            telemetry.addData("Regression Solved Pitch", "%.3f", shooterSystem.getCalculatedRegressionPitch());
            telemetry.addData("Regression Solved RPM", "%.0f", shooterSystem.getCalculatedRegressionRPM());
            telemetry.addData("Cluster Bearing", "%.1f deg", shooterSystem.getLastClusterBearing());
            telemetry.addData("Tag Spacing (px)", "%.1f", lastTagSpacing);

            telemetry.addLine("\n===== TURRET STATUS =====");
            telemetry.addData("Turret Aim Offset", "%.1f px", turretTracker.getAimOffsetPixels());
            telemetry.addData("Turret Position", "%.1f ticks", turretTracker.getTurretPosition());
            telemetry.addData("Turret State", turretTracker.getTrackerState());

            // Dashboard Packet for real-time visualization
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Distance (in)", shooterSystem.getLastEstimatedDist());
            packet.put("Base Pitch", shooterSystem.getBasePitch());
            packet.put("Compensated Pitch", shooterSystem.getCompensatedPitch());
            packet.put("Target RPM", shooterSystem.getActiveTargetRPM());
            packet.put("Current RPM", shooterSystem.getCurrentRPM());
            packet.put("Turret Aim Offset (px)", turretTracker.getAimOffsetPixels());
            packet.put("Turret Pos Ticks", turretTracker.getTurretPosition());
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }

        // Clean up on stop
        shooterSystem.stop();
        turretTracker.stop();
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (transferMotor != null) transferMotor.setPower(0);
        if (visionPortal != null) visionPortal.close();
    }

    private void initAprilTag() {
        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                    .setNumThreads(4)
                    .build();
            aprilTag.setDecimation(1.0f);

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(640, 480))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception ignored) {}
    }

    private double updateVisionAndRegression() {
        double pixelErrorX = -99999;
        if (aprilTag == null) return pixelErrorX;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) return pixelErrorX;

        int audienceStart = BLUE_ALLIANCE ? 38 : 34;
        int farStart = BLUE_ALLIANCE ? 42 : 30;

        int audienceCount = 0;
        int farCount = 0;
        for (AprilTagDetection detection : detections) {
            if (detection.id >= audienceStart && detection.id <= audienceStart + 3) {
                audienceCount++;
            } else if (detection.id >= farStart && detection.id <= farStart + 3) {
                farCount++;
            }
        }

        boolean audienceSide = audienceCount >= farCount;
        int targetClusterStart = audienceSide ? audienceStart : farStart;
        lastDetectedCell = audienceSide ? "AUDIENCE" : "FAR";

        // Evaluate 2D regression (Pitch + RPM)
        shooterSystem.updateClusterRegression(detections, targetClusterStart, lastTagSpacing);
        if (shooterSystem.getLastPerceivedSpacing() > 0) {
            lastTagSpacing = shooterSystem.getLastPerceivedSpacing();
        }

        // If button X pressed, re-engage automatic regression values
        if (gamepad1.x) {
            shooterSystem.setBasePitch(shooterSystem.getCalculatedRegressionPitch());
            shooterSystem.setTargetRPM(shooterSystem.getCalculatedRegressionRPM());
        }

        if (shooterSystem.getLastCentreX() > 0) {
            pixelErrorX = 320.0 - shooterSystem.getLastCentreX();
        }

        return pixelErrorX;
    }
}
