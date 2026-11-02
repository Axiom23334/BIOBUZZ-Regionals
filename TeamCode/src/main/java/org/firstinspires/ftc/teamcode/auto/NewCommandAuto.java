package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretTracker;
import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Regionals Autonomous", group = "Comp")
public class NewCommandAuto extends LinearOpMode {

    // Follower & Odometry
    private Follower follower;

    // Vision
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Mechanism Hardware
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private Servo flipper;

    // Dedicated Shooter System with velocity compensation & regression
    private ShooterSystem shooterSystem;

    // Dedicated Turret Tracker
    private TurretTracker turretTracker;

    // Flipper servo positions
    public static double FLIPPER_DOWN = 0.0;
    public static double FLIPPER_UP = 1.0;
    public static double FLIPPER_INTAKE = 0.5;

    // Default autonomous hood/pitch position fallback
    public static double AUTO_BASE_PITCH = 0.45;

    // Alliance configuration
    private static final boolean BLUE_ALLIANCE = true;
    private double lastTagSpacing = 40.0;

    // Autonomous Poses & Path
    private final Pose startPose = new Pose(9, 111, Math.toRadians(-90));
    private final Pose targetPose = new Pose(24, 111, Math.toRadians(-90));
    private PathChain pathChain;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Mechanism Hardware
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        flipper = hardwareMap.get(Servo.class, "flipper");

        // Initialize Shooter System with hood velocity compensation & regression
        shooterSystem = new ShooterSystem(hardwareMap);
        shooterSystem.setBasePitch(AUTO_BASE_PITCH);

        // Initialize Turret Tracker with Pinpoint IMU (heading-only, pods not required)
        turretTracker = new TurretTracker(hardwareMap);
        try {
            GoBildaPinpointDriver pin = hardwareMap.get(GoBildaPinpointDriver.class, "pin");
            turretTracker.setPin(pin);
        } catch (Exception ignored) {}

        // Initialize AprilTag vision for target detection during auto
        initAprilTag();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();

        telemetry.addLine("Auto Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // 1. Follow Path to target position
        follower.followPath(pathChain, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateVisionAndShooter();
            telemetryOutput();
        }

        // 2. Intaking state: Run intake motor & drop flipper with special intake command
        intakeMotor.setPower(1.0);
        transferMotor.setPower(0.0);
        flipper.setPosition(FLIPPER_INTAKE);
        waitMillis(1000);

        // 3. Idle state: Stop intake and transfer
        intakeMotor.setPower(0.0);
        transferMotor.setPower(0.0);
        flipper.setPosition(FLIPPER_UP);
        waitMillis(500);

        // 4. Shooting state: Run both intake and transfer to feed into 3500 RPM shooter
        intakeMotor.setPower(1.0);
        transferMotor.setPower(1.0);
        waitMillis(1500);

        // 5. After shooting, go back to intaking again
        intakeMotor.setPower(1.0);
        transferMotor.setPower(0.0);
        waitMillis(1000);

        // Stop all mechanisms
        stopMechanisms();
    }

    private void waitMillis(long ms) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < ms) {
            follower.update();
            updateVisionAndShooter();
            telemetryOutput();
        }
    }

    private void stopMechanisms() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        shooterSystem.stop();
        turretTracker.stop();
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void initAprilTag() {
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
    }

    private void updateVisionAndShooter() {
        double pixelErrorX = -99999;
        String cell = "NONE";

        if (aprilTag != null) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
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
            cell = audienceSide ? "AUDIENCE" : "FAR";

            shooterSystem.updateClusterRegression(detections, targetClusterStart, lastTagSpacing);
            if (shooterSystem.getLastPerceivedSpacing() > 0) {
                lastTagSpacing = shooterSystem.getLastPerceivedSpacing();
            }

            // Robust centrepoint from ShooterSystem's cluster geometry solver
            if (shooterSystem.getLastCentreX() > 0) {
                pixelErrorX = 320.0 - shooterSystem.getLastCentreX();
            }
        }

        // Update Turret Tracker with vision error & Pinpoint heading fallback
        turretTracker.update(pixelErrorX, cell);

        shooterSystem.update();
    }

    private void telemetryOutput() {
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooterSystem.getCurrentRPM(), ShooterSystem.TARGET_RPM);
        telemetry.addData("Base Pitch", "%.3f", shooterSystem.getBasePitch());
        telemetry.addData("Compensated Pitch", "%.3f", shooterSystem.getCompensatedPitch());
        telemetry.addData("RPM Error", "%.0f", shooterSystem.getRpmError());
        telemetry.addData("Regression Active", shooterSystem.isRegressionActive());
        telemetry.addData("Estimated Dist", "%.1f in", shooterSystem.getLastEstimatedDist());
        telemetry.addData("Cluster Bearing", "%.1f deg", shooterSystem.getLastClusterBearing());
        telemetry.addData("Tag Spacing", "%.1f px", lastTagSpacing);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
