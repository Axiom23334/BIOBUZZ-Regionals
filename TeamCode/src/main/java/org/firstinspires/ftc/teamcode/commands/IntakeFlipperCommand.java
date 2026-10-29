package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.FlipperSubsystem;

public class IntakeFlipperCommand extends CommandBase {
    private FlipperSubsystem flipperSubsystem;
    private double targetPosition = 0;

    public IntakeFlipperCommand(FlipperSubsystem flipperSubsystem) {
        this(flipperSubsystem, FlipperSubsystem.INTAKE_POSITION);
    }

    public IntakeFlipperCommand(FlipperSubsystem flipperSubsystem, double targetPosition) {
        this.flipperSubsystem = flipperSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(flipperSubsystem);
    }

    @Override
    public void initialize() {
        flipperSubsystem.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
