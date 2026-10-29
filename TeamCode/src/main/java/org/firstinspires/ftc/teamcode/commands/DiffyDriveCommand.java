package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DiffyDriveSubsystem;

import java.util.function.DoubleSupplier;

public class DiffyDriveCommand extends CommandBase {
    private final DiffyDriveSubsystem driveSubsystem;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    public DiffyDriveCommand(DiffyDriveSubsystem driveSubsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.driveSubsystem = driveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}
