package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Regionals TeleOp", group = "Comp")
public class RegionalsTeleOp extends LinearOpMode {

    // Drive Motors (Diffy)
    private DcMotorEx frontRightMotor; // leftMotor1 in diffyCode
    private DcMotorEx backRightMotor;  // leftMotor2 in diffyCode
    private DcMotorEx frontLeftMotor;  // rightMotor1 in diffyCode
    private DcMotorEx backLeftMotor;   // rightMotor2 in diffyCode
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    // Mechanism Motors & Servos
    private DcMotorEx turretMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private DcMotorEx shooterMotor;
    private Servo flipper;

    private Servo pitch;

    // Diffy drive tuning & readiness
    public static double kP1 = 0.7;
    public static double kP2 = 0.7;
    private boolean diffy1Ready = false;
    private boolean diffy2Ready = false;

    // Flipper servo positions
    public static double FLIPPER_DOWN = 0.0;
    public static double FLIPPER_UP = 1.0;

    // Shooter PID (target 3500 RPM)
    private final PIDFController shooterPid = new PIDFController(0.0008, 0, 0, 0.00025);
    public static double SHOOTER_TICKS_PER_REV = 28.0;
    public static double TARGET_SHOOTER_RPM = 3500.0;

    // State Machine States
    public enum RobotState {
        INTAKING,
        IDLE,
        SHOOTING
    }
    long lastLoopTime = 0;
    private RobotState currentState = RobotState.INTAKING;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private static final double SHOOT_DURATION_SECONDS = 1.0;

    // Button edge detection for bumper cycling and flipper
    private boolean lastR1 = false;
    private boolean lastL1 = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadUp = false;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Init Subsystems
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-170.5, 42.023);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        swerve.updateOdo(odo);
        // Shooting Status



        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        // Initialize Drive Motors
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");

        // Initialize Mechanism Hardware
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        flipper = hardwareMap.get(Servo.class, "flipper");

        pitch = hardwareMap.get(Servo.class, "pitch");

        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        double angle = 0;

        waitForStart();
        if (isStopRequested()) return;

        stateTimer.reset();

        while (opModeIsActive()) {

            telemetryAprilTag();

            if (gamepad1.a){
                angle = angle + 0.001;
            } else if (gamepad1.y){
                angle = angle - 0.001;
            }

            telemetry.addData("angle", angle);
            pitch.setPosition(angle);

            // ------------------- 2. STATE TRANSITIONS (R1 / L1) -------------------
            boolean currentR1 = gamepad1.right_bumper;
            boolean currentL1 = gamepad1.left_bumper;

            // R1: forward transition (INTAKING -> IDLE -> SHOOTING)
            if (currentL1) {
                currentState = RobotState.INTAKING;
                stateTimer.reset();
            } else if (currentR1){
                currentState = RobotState.SHOOTING;
                stateTimer.reset();
            } else{
                currentState = RobotState.IDLE;
                stateTimer.reset();
            }

            // ------------------- 3. MANUAL FLIPPER CONTROLS -------------------
            boolean currentDpadDown = gamepad1.dpad_down;
            boolean currentDpadUp = gamepad1.dpad_up;

            if (currentDpadDown && !lastDpadDown) {
                flipper.setPosition(FLIPPER_DOWN);
            }
            if (currentDpadUp && !lastDpadUp) {
                flipper.setPosition(FLIPPER_UP);
            }

            lastDpadDown = currentDpadDown;
            lastDpadUp = currentDpadUp;

            // ------------------- 4. MECHANISM CONTROL & STATE MACHINE -------------------
            // Continuous shooter PID targeting 3500 RPM

            if (!gamepad1.left_trigger_pressed){
                double currentTicksPerSec = shooterMotor.getVelocity();
                double currentRPM = (-currentTicksPerSec * 60.0) / SHOOTER_TICKS_PER_REV;
                shooterPid.setSetPoint(TARGET_SHOOTER_RPM);
                double shooterPower = shooterPid.calculate(currentRPM);
                shooterMotor.setPower(Range.clip(shooterPower, -1.0, 1.0));
                telemetry.addData("Shooter RPM", "%.0f / %.0f", currentRPM, TARGET_SHOOTER_RPM);
            } else{
                shooterMotor.setPower(0);
            }
            telemetry.addData("angle", angle);

            // State behaviors
            switch (currentState) {
                case INTAKING:
                    // On intake: run intakeMotor and not transferMotor (no automatic flipper drop)
                    intakeMotor.setPower(1.0);
                    transferMotor.setPower(0);
                    break;

                case IDLE:
                    // Idle state between intake and shooting: nothing runs
                    intakeMotor.setPower(0.0);
                    transferMotor.setPower(0.0);
                    break;

                case SHOOTING:
                    // On shooting: run both intake and transfer to feed shooter
                    intakeMotor.setPower(1.0);
                    transferMotor.setPower(1.0);
                    break;
            }
            odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);

            // 1. Inputs
            double driveScale = SwerveTeleOpConfig.DRIVE_SPEED_SCALAR;
            double rotScale = SwerveTeleOpConfig.ROTATION_SPEED_SCALAR;

            double strafe = gamepad1.left_stick_x * driveScale;
            double forward = -gamepad1.left_stick_y * driveScale;


            double rot = -gamepad1.right_stick_x * rotScale;

            // 2. Drive Command
            // We pass ALL config values here so they update live!
            swerve.drive(
                    strafe, forward, rot,
                    SwerveTeleOpConfig.module1Adjust,
                    SwerveTeleOpConfig.module2Adjust,
                    SwerveTeleOpConfig.module3Adjust,
                    SwerveTeleOpConfig.Kp,
                    SwerveTeleOpConfig.Kd,
                    SwerveTeleOpConfig.Ki,
                    SwerveTeleOpConfig.Kf,
                    SwerveTeleOpConfig.Kl,
                    SwerveTeleOpConfig.FIELD_CENTRIC,
                    SwerveTeleOpConfig.IMU_POLARITY,
                    SwerveTeleOpConfig.ROBOT_RADIUS
            );

            // 3. Reset IMU
            if (gamepad1.options || gamepad1.start) {
                swerve.resetIMU();
                gamepad1.rumble(500);
            }



            long currentTime = System.currentTimeMillis();
            long loopTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;


        }

        // Clean up on stop
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        shooterMotor.setPower(0);
        turretMotor.setPower(0);
    }
    private void initAprilTag() {
        // Create AprilTag processor configured with ThriftyCam / ThriftestCam lens intrinsics
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506) // ThriftyCam 640x480 calibration
                .setNumThreads(4)                                      // Parallel processing for fast detection
                .build();

        // Set decimation to 2.0f for higher speed and range accuracy
        aprilTag.setDecimation(2.0f);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // 640x480 resolution matches ThriftyCam intrinsics
        builder.setCameraResolution(new Size(640, 480));

        // StreamFormat.MJPEG eliminates USB 2.0 YUY2 bandwidth bottlenecks and unlocks 60+ FPS
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private void telemetryAprilTag() {
        if (visionPortal != null) {
            telemetry.addData("VisionPortal FPS", "%.1f", visionPortal.getFps());
        }
        if (aprilTag != null) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
        }
    }
}
