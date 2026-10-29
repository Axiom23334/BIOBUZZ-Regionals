package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.IntakeFlipperCommand;
import org.firstinspires.ftc.teamcode.commands.StateMachineCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlipperSubsystem;

@Autonomous(name = "New SolversLib Auto", group = "Autonomous")
public class NewCommandAuto extends CommandOpMode {

    private Follower follower;
    private TelemetryData telemetryData;
    private StateMachineCommand stateMachineCommand;
    private FlipperSubsystem flipperSubsystem;

    private final Pose startPose = new Pose(9, 111, Math.toRadians(-90));
    private final Pose targetPose = new Pose(24, 111, Math.toRadians(-90));

    private PathChain pathChain;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        telemetryData = new TelemetryData(telemetry);

        flipperSubsystem = new FlipperSubsystem(hardwareMap);
        stateMachineCommand = new StateMachineCommand();

        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();

        // Sequence: Follow path -> Intaking state -> Flipper Intake Command -> Launching state -> Wait
        schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, pathChain),
                        new InstantCommand(() -> stateMachineCommand.setState(StateMachineCommand.State.INTAKING)),
                        new IntakeFlipperCommand(flipperSubsystem),
                        new WaitCommand(500),
                        new InstantCommand(() -> stateMachineCommand.setState(StateMachineCommand.State.LAUNCHING)),
                        new WaitCommand(1000)
                )
        );
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        telemetryData.addData("State Machine State", stateMachineCommand.getCurrentState());
        telemetryData.addData("Follower Busy", follower.isBusy());
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}
