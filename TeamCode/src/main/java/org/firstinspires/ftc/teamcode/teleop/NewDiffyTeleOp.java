package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DiffyDriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeFlipperCommand;
import org.firstinspires.ftc.teamcode.commands.StateMachineCommand;
import org.firstinspires.ftc.teamcode.subsystems.DiffyDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlipperSubsystem;

@TeleOp(name = "New Diffy TeleOp", group = "TeleOp")
public class NewDiffyTeleOp extends CommandOpMode {

    private DiffyDriveSubsystem driveSubsystem;
    private FlipperSubsystem flipperSubsystem;
    private StateMachineCommand stateMachineCommand;
    private GamepadEx driverGamepad;

    @Override
    public void initialize() {
        super.reset();

        driveSubsystem = new DiffyDriveSubsystem(hardwareMap);
        flipperSubsystem = new FlipperSubsystem(hardwareMap);
        stateMachineCommand = new StateMachineCommand();
        driverGamepad = new GamepadEx(gamepad1);

        // Schedule continuous state machine command
        schedule(stateMachineCommand);

        // Register default drive command using joystick inputs from gamepad1
        driveSubsystem.setDefaultCommand(
                new DiffyDriveCommand(
                        driveSubsystem,
                        () -> gamepad1.left_stick_y,
                        () -> gamepad1.left_stick_x
                )
        );

        // State machine state switching with R1 (RIGHT_BUMPER) and L1 (LEFT_BUMPER)
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(stateMachineCommand::nextState);

        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(stateMachineCommand::previousState);

        // Separate command for Intaking (flipper) bound to Button X
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenActive(new IntakeFlipperCommand(flipperSubsystem));
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("State Machine State", stateMachineCommand.getCurrentState());
        telemetry.addData("Diffy 1 Ready", driveSubsystem.diffy1Ready);
        telemetry.addData("Diffy 2 Ready", driveSubsystem.diffy2Ready);
        telemetry.update();
    }
}
