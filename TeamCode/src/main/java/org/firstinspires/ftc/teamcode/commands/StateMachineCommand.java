package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;

/**
 * SolversLib Command containing a finite state machine for robot states:
 * Idle, Intaking, Launching.
 */
public class StateMachineCommand extends CommandBase {

    public enum State {
        IDLE,
        INTAKING,
        LAUNCHING
    }

    private State currentState = State.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    public StateMachineCommand() {
    }

    public StateMachineCommand(Subsystem... requirements) {
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        currentState = State.IDLE;
        stateTimer.reset();
    }

    @Override
    public void execute() {
        switch (currentState) {
            case IDLE:
                // Robot idle state
                break;

            case INTAKING:
                // Robot intaking state
                break;

            case LAUNCHING:
                // Robot launching state
                break;

            default:
                break;
        }
    }

    public void nextState() {
        switch (currentState) {
            case IDLE:
                currentState = State.INTAKING;
                break;
            case INTAKING:
            case LAUNCHING:
                currentState = State.LAUNCHING;
                break;
        }
        stateTimer.reset();
    }

    public void previousState() {
        switch (currentState) {
            case LAUNCHING:
                currentState = State.INTAKING;
                break;
            case INTAKING:
            case IDLE:
                currentState = State.IDLE;
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
        return false; // Runs continuously in TeleOp
    }

    @Override
    public void end(boolean interrupted) {
        currentState = State.IDLE;
    }

    public State getCurrentState() {
        return currentState;
    }

    public double getStateTimeSeconds() {
        return stateTimer.seconds();
    }
}
