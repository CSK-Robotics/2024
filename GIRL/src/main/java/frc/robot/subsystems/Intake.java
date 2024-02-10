package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public class State {
        public double rollerSpeed;

        public State(double rollerSpeed) {
            this.rollerSpeed = rollerSpeed;
        }
    }

    private State setpoint;

    private final CANSparkMax m_rollers = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);
    private final DigitalInput m_noteDetector = new DigitalInput(0);

    @Override
    public void periodic() {
        m_rollers.set(setpoint.rollerSpeed);
    }

    private void setState(State desiredState) {
        this.setpoint = desiredState;
    }

    // true is to intake, false is to outtake
    public Command run(boolean direction) {
        State desiredState = new State(direction ? 0.5 : -0.5);
        return this.run(() -> {
            setState(desiredState);
        }).until(() -> direction ? hasGamePiece() : !hasGamePiece());
    }

    // true is to pass to shooter, false is to recieve from shooter
    public Command handoff(boolean direction) {
        State desiredState = new State(direction ? 0.5 : -0.5);
        return this.run(() -> {
            setState(desiredState);
        }).until(() -> direction ? !hasGamePiece() : hasGamePiece());
    }

    public boolean hasGamePiece() {
        return m_noteDetector.get();
    }
}
