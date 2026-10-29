package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.FlipperSubsystem;

public class FlipperUpCommand extends CommandBase {
    private final FlipperSubsystem flipperSubsystem;

    public FlipperUpCommand(FlipperSubsystem flipperSubsystem) {
        this.flipperSubsystem = flipperSubsystem;
        addRequirements(flipperSubsystem);
    }

    @Override
    public void initialize() {
        flipperSubsystem.flipperUp();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
