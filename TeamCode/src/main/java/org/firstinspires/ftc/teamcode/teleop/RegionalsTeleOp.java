package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DiffyDriveCommand;
import org.firstinspires.ftc.teamcode.commands.FlipperDownCommand;
import org.firstinspires.ftc.teamcode.commands.FlipperUpCommand;
import org.firstinspires.ftc.teamcode.commands.StateMachineCommand;
import org.firstinspires.ftc.teamcode.subsystems.DiffyDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlipperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MechanismSubsystem;

@TeleOp(name = "Regionals TeleOp", group = "Comp")
public class RegionalsTeleOp extends CommandOpMode {

    // Subsystems
    private DiffyDriveSubsystem driveSubsystem;
    private FlipperSubsystem flipperSubsystem;
    private MechanismSubsystem mechanismSubsystem;

    // Mechanism motors imported into OpMode
    private DcMotorEx turretMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private DcMotorEx shooterMotor;

    private StateMachineCommand stateMachineCommand;
    private GamepadEx driverGamepad;

    @Override
    public void initialize() {
        super.reset();

        driveSubsystem = new DiffyDriveSubsystem(hardwareMap);
        flipperSubsystem = new FlipperSubsystem(hardwareMap);
        mechanismSubsystem = new MechanismSubsystem(hardwareMap);

        // Retrieve motors for direct inspection / telemetry
        turretMotor = mechanismSubsystem.getTurretMotor();
        intakeMotor = mechanismSubsystem.getIntakeMotor();
        transferMotor = mechanismSubsystem.getTransferMotor();
        shooterMotor = mechanismSubsystem.getShooterMotor();

        stateMachineCommand = new StateMachineCommand(mechanismSubsystem);
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

        // Driver changes between states with R1 (forward) and L1 (backward)
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(stateMachineCommand::nextState);

        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(stateMachineCommand::previousState);

        // Driver manual flipper control: flipper down (D-pad Down) and flipper up (D-pad Up)
        // IntakeFlipperCommand is NOT bound here as it is reserved for auto only.
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenActive(new FlipperDownCommand(flipperSubsystem));

        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenActive(new FlipperUpCommand(flipperSubsystem));
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("State", stateMachineCommand.getCurrentState());
        if (shooterMotor != null) {
            double rpm = (shooterMotor.getVelocity() * 60.0) / MechanismSubsystem.SHOOTER_TICKS_PER_REV;
            telemetry.addData("Shooter RPM", "%.0f / 3500", rpm);
        }
        telemetry.addData("Diffy 1 Ready", driveSubsystem.diffy1Ready);
        telemetry.addData("Diffy 2 Ready", driveSubsystem.diffy2Ready);
        telemetry.update();
    }
}
