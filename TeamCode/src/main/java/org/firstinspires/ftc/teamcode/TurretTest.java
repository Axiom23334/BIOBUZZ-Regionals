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
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;
import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

@Config
@TeleOp(name = "Utility: Turret Test", group = "Utility")
public class TurretTest extends LinearOpMode {

    // Hardware
    private DcMotorEx turretMotor;
    private GoBildaPinpointDriver odo;

    // Vision & Tracker
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private TurretTracker turretTracker;

    private boolean lastXButton = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Telemetry to phone DS & FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize Pinpoint driver if present
        try {
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        } catch (Exception e) {
            odo = null;
        }

        // Initialize TurretTracker
        turretTracker = new TurretTracker(hardwareMap);
        if (odo != null) {
            turretTracker.setOdo(odo);
        }

        // Initialize AprilTag Vision
        initAprilTag();

        telemetry.addLine("Turret Automatic Tracking Test Initialized.");
        telemetry.addLine("Turret automatically aims using Camera Vision & Robot Rotation (Pinpoint IMU).");
        telemetry.addLine("Press Button X to zero encoder position.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double currentTicks = turretMotor.getCurrentPosition();
            double pixelErrorX = getAprilTagPixelError();

            // Zero Encoder on Button X
            boolean currentX = gamepad1.x;
            if (currentX && !lastXButton) {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            lastXButton = currentX;

            // Automatic tracking update (Camera Vision error + Pinpoint robot rotation compensation)
            double appliedPower = turretTracker.update(pixelErrorX, "TEST");

            // Telemetry Output
            telemetry.addLine("===== AUTOMATIC TURRET TRACKING =====");
            telemetry.addData("Tracker State", turretTracker.isPinpointAssisted() ? "PINPOINT_HEADING" : (turretTracker.isTracking() ? "VISION" : "SEARCHING / IDLE"));
            telemetry.addData("Turret Position", "%.1f ticks", currentTicks);
            telemetry.addData("Applied Power", "%.3f", appliedPower);
            telemetry.addData("Motor Velocity", "%.1f ticks/s", turretMotor.getVelocity());

            telemetry.addLine("\n===== VISION & BOT ROTATION =====");
            telemetry.addData("AprilTag Error X", pixelErrorX != -99999 ? String.format(Locale.US, "%.1f px", pixelErrorX) : "NO TAG");
            telemetry.addData("Pinpoint Bot Heading", "%.1f deg", turretTracker.updateHeadingFromPinpoint());
            telemetry.addData("Target World Heading", "%.1f deg", turretTracker.getTargetTurretHeading());

            // Send Dashboard Plotting Packet
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Turret Pos Ticks", currentTicks);
            packet.put("Turret Power", appliedPower);
            packet.put("Pixel Error X", pixelErrorX);
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }

        turretMotor.setPower(0);
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void initAprilTag() {
        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                    .setNumThreads(4)
                    .build();
            aprilTag.setDecimation(2.0f);

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(640, 480))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception e) {
            telemetry.addData("Vision Warning", "Webcam 1 not found or failed to initialize.");
        }
    }

    private double getAprilTagPixelError() {
        if (aprilTag == null) return -99999;
        try {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (detections == null || detections.isEmpty()) return -99999;

            double sumX = 0;
            int count = 0;
            for (AprilTagDetection detection : detections) {
                sumX += detection.center.x;
                count++;
            }
            if (count == 0) return -99999;
            double avgX = sumX / count;
            return 320.0 - avgX;
        } catch (Exception e) {
            return -99999;
        }
    }
}
