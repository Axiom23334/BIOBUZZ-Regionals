package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.MechanismSubsystem;

/**
 * On transfer command: run both intakeMotor and transferMotor.
 */
public class TransferCommand extends CommandBase {
    private final MechanismSubsystem mechanismSubsystem;

    public TransferCommand(MechanismSubsystem mechanismSubsystem) {
        this.mechanismSubsystem = mechanismSubsystem;
        addRequirements(mechanismSubsystem);
    }

    @Override
    public void execute() {
        if (mechanismSubsystem != null) {
            mechanismSubsystem.runTransfer();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (mechanismSubsystem != null) {
            mechanismSubsystem.stopAll();
        }
    }
}
