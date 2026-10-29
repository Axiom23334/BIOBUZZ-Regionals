package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.FlipperSubsystem;

public class FlipperDownCommand extends CommandBase {
    private final FlipperSubsystem flipperSubsystem;

    public FlipperDownCommand(FlipperSubsystem flipperSubsystem) {
        this.flipperSubsystem = flipperSubsystem;
        addRequirements(flipperSubsystem);
    }

    @Override
    public void initialize() {
        flipperSubsystem.flipperDown();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
