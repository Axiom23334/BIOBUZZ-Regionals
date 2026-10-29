package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.MechanismSubsystem;

/**
 * SolversLib Command containing a finite state machine for robot states:
 * INTAKING, IDLE (between intake and shooting where nothing runs), SHOOTING (launching at 3500 RPM, then transitions back to INTAKING).
 * Shooter motor runs continuously on a PID loop at 3500 RPM throughout execution.
 * Flipper is NOT automatically dropped on intake; flipper is handled by separate commands.
 */
public class StateMachineCommand extends CommandBase {

    public enum State {
        INTAKING,
        IDLE,
        SHOOTING
    }

    private State currentState = State.INTAKING;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private static final double SHOOT_DURATION_SECONDS = 1.0;
    public static final double TARGET_SHOOTER_RPM = 3500.0;

    private final MechanismSubsystem mechanismSubsystem;

    public StateMachineCommand(MechanismSubsystem mechanismSubsystem) {
        this.mechanismSubsystem = mechanismSubsystem;
        if (mechanismSubsystem != null) {
            addRequirements(mechanismSubsystem);
        }
    }

    @Override
    public void initialize() {
        currentState = State.INTAKING;
        stateTimer.reset();
    }

    @Override
    public void execute() {
        // Run the shooter motor continuously on a PID loop at 3500 RPM
        if (mechanismSubsystem != null) {
            mechanismSubsystem.runShooterPID(TARGET_SHOOTER_RPM);
        }

        switch (currentState) {
            case INTAKING:
                // On intake: run intakeMotor and not transferMotor (flipper not moved automatically)
                if (mechanismSubsystem != null) {
                    mechanismSubsystem.runIntakeOnly();
                }
                break;

            case IDLE:
                // Idle state between intake and shooting: nothing runs (shooter PID runs continuously)
                if (mechanismSubsystem != null) {
                    mechanismSubsystem.stopIntakeAndTransfer();
                }
                break;

            case SHOOTING:
                // On shooting: run both intakeMotor and transferMotor to feed into shooter
                if (mechanismSubsystem != null) {
                    mechanismSubsystem.runTransfer();
                }

                // After shooting, go back to intaking again
                if (stateTimer.seconds() >= SHOOT_DURATION_SECONDS) {
                    currentState = State.INTAKING;
                    stateTimer.reset();
                }
                break;

            default:
                break;
        }
    }

    /**
     * Driver advances state forward (R1): INTAKING -> IDLE -> SHOOTING
     */
    public void nextState() {
        switch (currentState) {
            case INTAKING:
                currentState = State.IDLE;
                break;
            case IDLE:
                currentState = State.SHOOTING;
                break;
            case SHOOTING:
                currentState = State.INTAKING;
                break;
        }
        stateTimer.reset();
    }

    /**
     * Driver steps state backward (L1): SHOOTING -> IDLE -> INTAKING
     */
    public void previousState() {
        switch (currentState) {
            case SHOOTING:
                currentState = State.IDLE;
                break;
            case IDLE:
                currentState = State.INTAKING;
                break;
            case INTAKING:
                currentState = State.SHOOTING;
                break;
        }
        stateTimer.reset();
    }

    public void setState(State state) {
        this.currentState = state;
        stateTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        currentState = State.IDLE;
        if (mechanismSubsystem != null) {
            mechanismSubsystem.stopAll();
        }
    }

    public State getCurrentState() {
        return currentState;
    }

    public double getStateTimeSeconds() {
        return stateTimer.seconds();
    }
}
