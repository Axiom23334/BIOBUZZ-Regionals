package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

@TeleOp(name = "Regionals TeleOp", group = "Comp")
public class RegionalsTeleOp extends LinearOpMode {

    // Drive Motors (Diffy)
    private DcMotorEx frontRightMotor; // leftMotor1 in diffyCode
    private DcMotorEx backRightMotor;  // leftMotor2 in diffyCode
    private DcMotorEx frontLeftMotor;  // rightMotor1 in diffyCode
    private DcMotorEx backLeftMotor;   // rightMotor2 in diffyCode

    // Mechanism Motors & Servos
    private DcMotorEx turretMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private DcMotorEx shooterMotor;
    private Servo flipper;

    // Diffy drive tuning & readiness
    public static double kP1 = 0.7;
    public static double kP2 = 0.7;
    private boolean diffy1Ready = false;
    private boolean diffy2Ready = false;

    // Flipper servo positions
    public static double FLIPPER_DOWN = 0.0;
    public static double FLIPPER_UP = 1.0;

    // Shooter PID (target 3500 RPM)
    private final PIDFController shooterPid = new PIDFController(0.0008, 0.00005, 0.00001, 0.00025);
    public static double SHOOTER_TICKS_PER_REV = 28.0;
    public static double TARGET_SHOOTER_RPM = 3500.0;

    // State Machine States
    public enum RobotState {
        INTAKING,
        IDLE,
        SHOOTING
    }

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

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        stateTimer.reset();

        while (opModeIsActive()) {
            // ------------------- 1. DIFFY DRIVEBASE -------------------
            double y = gamepad1.left_stick_x;
            double x = gamepad1.left_stick_y;

            double currentTick1 = backRightMotor.getCurrentPosition();
            double currentAngle1 = ((currentTick1 / 6260.5) * Math.PI + Math.PI / 2);
            driveDiffy(x, y, frontLeftMotor, backLeftMotor, currentAngle1, kP1, 1);

            double currentTick2 = frontRightMotor.getCurrentPosition();
            double currentAngle2 = ((currentTick2 / 6260.5) * Math.PI + Math.PI / 2);
            driveDiffy(x, y, frontRightMotor, backRightMotor, currentAngle2, kP2, 2);

            // ------------------- 2. STATE TRANSITIONS (R1 / L1) -------------------
            boolean currentR1 = gamepad1.right_bumper;
            boolean currentL1 = gamepad1.left_bumper;

            // R1: forward transition (INTAKING -> IDLE -> SHOOTING)
            if (currentR1 && !lastR1) {
                switch (currentState) {
                    case INTAKING:
                        currentState = RobotState.IDLE;
                        break;
                    case IDLE:
                        currentState = RobotState.SHOOTING;
                        break;
                    case SHOOTING:
                        currentState = RobotState.INTAKING;
                        break;
                }
                stateTimer.reset();
            }

            // L1: backward transition (SHOOTING -> IDLE -> INTAKING)
            if (currentL1 && !lastL1) {
                switch (currentState) {
                    case SHOOTING:
                        currentState = RobotState.IDLE;
                        break;
                    case IDLE:
                        currentState = RobotState.INTAKING;
                        break;
                    case INTAKING:
                        currentState = RobotState.SHOOTING;
                        break;
                }
                stateTimer.reset();
            }

            lastR1 = currentR1;
            lastL1 = currentL1;

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
            double currentTicksPerSec = shooterMotor.getVelocity();
            double currentRPM = (currentTicksPerSec * 60.0) / SHOOTER_TICKS_PER_REV;
            shooterPid.setSetPoint(TARGET_SHOOTER_RPM);
            double shooterPower = shooterPid.calculate(currentRPM);
            shooterMotor.setPower(Range.clip(shooterPower, -1.0, 1.0));

            // State behaviors
            switch (currentState) {
                case INTAKING:
                    // On intake: run intakeMotor and not transferMotor (no automatic flipper drop)
                    intakeMotor.setPower(1.0);
                    transferMotor.setPower(0.0);
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

                    // After shooting duration, go back to intaking again
                    if (stateTimer.seconds() >= SHOOT_DURATION_SECONDS) {
                        currentState = RobotState.INTAKING;
                        stateTimer.reset();
                    }
                    break;
            }

            // ------------------- 5. TELEMETRY -------------------
            telemetry.addData("State", currentState);
            telemetry.addData("Shooter RPM", "%.0f / %.0f", currentRPM, TARGET_SHOOTER_RPM);
            telemetry.addData("Diffy 1 Ready", diffy1Ready);
            telemetry.addData("Diffy 2 Ready", diffy2Ready);
            telemetry.addData("Tick 1", currentTick1);
            telemetry.addData("Tick 2", currentTick2);
            telemetry.update();
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

    public void driveDiffy(double x, double y, DcMotorEx motor1, DcMotorEx motor2, double currentAngle, double kP, double diffy) {
        double targetAngle = Math.atan2(x, y);
        double error = targetAngle - currentAngle;

        double speed = 0;
        double power = 0;
        if (Math.abs(error) > 0.2) {
            motor1.setPower(-Range.clip(error * kP, -1, 1));
            motor2.setPower(-Range.clip(error * kP, -1, 1));
            if (diffy == 1) {
                diffy1Ready = false;
            }
            if (diffy == 2) {
                diffy2Ready = false;
            }
        } else {
            if (diffy == 1) {
                diffy1Ready = true;
            }
            if (diffy == 2) {
                diffy2Ready = true;
            }

            if (diffy1Ready && diffy2Ready) {
                power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
                speed = power * ((1 + ((double) 46 / 11)) * 28);
                motor1.setPower(-0);
                motor2.setPower(0);
            }
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Angle", targetAngle);
        packet.put("Current Angle", currentAngle);
        packet.put("Error", error);
        packet.put("speed", power);
        packet.put("y", y);
        dashboard.sendTelemetryPacket(packet);
    }
}
