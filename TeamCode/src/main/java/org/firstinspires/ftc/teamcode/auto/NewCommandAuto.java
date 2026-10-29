package org.firstinspires.ftc.teamcode.auto;

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
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Regionals Autonomous", group = "Comp")
public class NewCommandAuto extends LinearOpMode {

    // Follower & Odometry
    private Follower follower;

    // Mechanism Hardware
    private DcMotorEx turretMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private DcMotorEx shooterMotor;
    private Servo flipper;

    // Flipper servo positions
    public static double FLIPPER_DOWN = 0.0;
    public static double FLIPPER_UP = 1.0;
    public static double FLIPPER_INTAKE = 0.5;

    // Shooter PID (target 3500 RPM)
    private final PIDFController shooterPid = new PIDFController(0.0008, 0.00005, 0.00001, 0.00025);
    public static double SHOOTER_TICKS_PER_REV = 28.0;
    public static double TARGET_SHOOTER_RPM = 3500.0;

    // Autonomous Poses & Path
    private final Pose startPose = new Pose(9, 111, Math.toRadians(-90));
    private final Pose targetPose = new Pose(24, 111, Math.toRadians(-90));
    private PathChain pathChain;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Mechanism Hardware
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        flipper = hardwareMap.get(Servo.class, "flipper");

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
            updateShooterPID();
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

    private void updateShooterPID() {
        if (shooterMotor != null) {
            double currentTicksPerSec = shooterMotor.getVelocity();
            double currentRPM = (currentTicksPerSec * 60.0) / SHOOTER_TICKS_PER_REV;
            shooterPid.setSetPoint(TARGET_SHOOTER_RPM);
            double shooterPower = shooterPid.calculate(currentRPM);
            shooterMotor.setPower(Range.clip(shooterPower, -1.0, 1.0));
        }
    }

    private void waitMillis(long ms) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < ms) {
            follower.update();
            updateShooterPID();
            telemetryOutput();
        }
    }

    private void stopMechanisms() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        shooterMotor.setPower(0);
        turretMotor.setPower(0);
    }

    private void telemetryOutput() {
        double currentRPM = (shooterMotor.getVelocity() * 60.0) / SHOOTER_TICKS_PER_REV;
        telemetry.addData("Shooter RPM", "%.0f / %.0f", currentRPM, TARGET_SHOOTER_RPM);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
