package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.IntakeFlipperCommand;
import org.firstinspires.ftc.teamcode.commands.StateMachineCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DiffyDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlipperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MechanismSubsystem;

@Autonomous(name = "Regionals Autonomous", group = "Autonomous")
public class NewCommandAuto extends CommandOpMode {

    private Follower follower;
    private TelemetryData telemetryData;

    // Subsystems
    private DiffyDriveSubsystem driveSubsystem;
    private MechanismSubsystem mechanismSubsystem;
    private FlipperSubsystem flipperSubsystem;
    private StateMachineCommand stateMachineCommand;

    // Mechanism motors imported into OpMode
    private DcMotorEx turretMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private DcMotorEx shooterMotor;

    private final Pose startPose = new Pose(9, 111, Math.toRadians(-90));
    private final Pose targetPose = new Pose(24, 111, Math.toRadians(-90));

    private PathChain pathChain;

    @Override
    public void initialize() {
        super.reset();

        // 1. Setup Diffy Drivebase subsystem (same hardware and math as diffyCode)
        driveSubsystem = new DiffyDriveSubsystem(hardwareMap);

        // 2. Initialize Follower using the Diffy Drivetrain
        follower = Constants.createFollower(hardwareMap, driveSubsystem);
        follower.setStartingPose(startPose);
        telemetryData = new TelemetryData(telemetry);

        mechanismSubsystem = new MechanismSubsystem(hardwareMap);
        flipperSubsystem = new FlipperSubsystem(hardwareMap);

        // Retrieve motors for OpMode
        turretMotor = mechanismSubsystem.getTurretMotor();
        intakeMotor = mechanismSubsystem.getIntakeMotor();
        transferMotor = mechanismSubsystem.getTransferMotor();
        shooterMotor = mechanismSubsystem.getShooterMotor();

        stateMachineCommand = new StateMachineCommand(mechanismSubsystem);

        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();

        // Autonomous sequence: Follow path -> Intake with IntakeFlipperCommand -> Shooting -> Return to Intaking
        schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, pathChain),
                        // Set state to INTAKING (runs intake motor, flipper NOT dropped automatically)
                        new InstantCommand(() -> stateMachineCommand.setState(StateMachineCommand.State.INTAKING)),
                        // Special IntakeFlipperCommand used specifically for auto
                        new IntakeFlipperCommand(flipperSubsystem),
                        new WaitCommand(1000),
                        // Intermediate IDLE state (nothing running except shooter PID)
                        new InstantCommand(() -> stateMachineCommand.setState(StateMachineCommand.State.IDLE)),
                        new WaitCommand(500),
                        // Trigger SHOOTING (feeds transfer motor, returns automatically to INTAKING after shot)
                        new InstantCommand(() -> stateMachineCommand.setState(StateMachineCommand.State.SHOOTING)),
                        new WaitCommand(1500)
                )
        );
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        // Keep shooter PID loop running during Autonomous
        if (mechanismSubsystem != null) {
            mechanismSubsystem.runShooterPID(StateMachineCommand.TARGET_SHOOTER_RPM);
        }

        telemetryData.addData("State Machine State", stateMachineCommand.getCurrentState());
        if (shooterMotor != null) {
            double rpm = (shooterMotor.getVelocity() * 60.0) / MechanismSubsystem.SHOOTER_TICKS_PER_REV;
            telemetryData.addData("Shooter RPM", String.format("%.0f / 3500", rpm));
        }
        telemetryData.addData("Diffy 1 Ready", driveSubsystem.diffy1Ready);
        telemetryData.addData("Diffy 2 Ready", driveSubsystem.diffy2Ready);
        telemetryData.addData("Follower Busy", follower.isBusy());
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}
