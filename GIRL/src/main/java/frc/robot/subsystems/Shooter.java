package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public enum Goal {
        SPEAKER, AMP, TRAP, HANDOFF, SOURCE, IDLE
    }

    public class State {
        public Goal goal;
        public double flyWheelSpeed;
        public double indexerSpeed;

        public State(Goal goal, double flyWheelSpeed, double indexerSpeed) {
            this.goal = Goal.IDLE;
            this.flyWheelSpeed = 0.0;
            this.indexerSpeed = 0.0;
        }
    }

    private State setpoint;
    private EventLoop m_loop;

    private final CANSparkFlex m_top = new CANSparkFlex(1, CANSparkMax.MotorType.kBrushless);
    private final CANSparkFlex m_bottom = new CANSparkFlex(1, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax m_indexer = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);
    private final DigitalInput m_noteDetector = new DigitalInput(0);

    @Override
    public void periodic() {
        m_top.set(setpoint.flyWheelSpeed);
        m_bottom.set(setpoint.flyWheelSpeed);
        m_indexer.set(setpoint.indexerSpeed);
    }

    private void setState(State desiredState) {
        this.setpoint = desiredState;
    }

    public void stop() {
        setState(new State(Goal.IDLE, 0.0, 0.0));
    }

    public void stop(Goal goal) {
        setState(new State(goal, 0.0, 0.0));
    }

    public Command shoot(double targetSpeed, Goal goal) {
        m_loop = new EventLoop();
        BooleanEvent primedEvent = new BooleanEvent(m_loop, () -> isPrimed()).debounce(0.1);
        primedEvent.negate().ifHigh(() -> setState(new State(goal, targetSpeed, 0.0)));
        primedEvent.rising().ifHigh(() -> setState(new State(goal, targetSpeed, targetSpeed)));
        return this.runEnd(() -> m_loop.poll(), this::stop).until(primedEvent.falling());
    }

    // true is to intake, false is to outtake
    public Command handoff(boolean direction) {
        State desiredState = new State(Goal.HANDOFF, 0.0, direction ? 0.5 : -0.5);
        return this.run(() -> {
            setState(desiredState);
        }).until(() -> direction ? hasGamePiece() : !hasGamePiece());
    }

    public Command source() {
        State desiredState = new State(Goal.HANDOFF, -0.5, -0.5);
        return this.runEnd(() -> {
            setState(desiredState);
        }, this::stop).until(this::hasGamePiece);
    }

    private boolean isPrimed() {
        return false;
    }

    public boolean hasGamePiece() {
        return m_noteDetector.get();
    }
}
