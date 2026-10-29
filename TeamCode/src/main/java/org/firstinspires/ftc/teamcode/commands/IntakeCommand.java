package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.MechanismSubsystem;

/**
 * On intake command: run intakeMotor and not transferMotor.
 * Note: Does NOT drop the flipper automatically. Flipper is controlled separately.
 */
public class IntakeCommand extends CommandBase {
    private final MechanismSubsystem mechanismSubsystem;

    public IntakeCommand(MechanismSubsystem mechanismSubsystem) {
        this.mechanismSubsystem = mechanismSubsystem;
        addRequirements(mechanismSubsystem);
    }

    @Override
    public void execute() {
        if (mechanismSubsystem != null) {
            mechanismSubsystem.runIntakeOnly();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (mechanismSubsystem != null) {
            mechanismSubsystem.stopIntakeAndTransfer();
        }
    }
}
