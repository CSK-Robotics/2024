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

    public Intake(DigitalInput detector) {
        // m_noteDetector = detector;
        setpoint = new State(0.0);
    }

    private State setpoint;

    private final CANSparkMax m_rollers = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
    // private final DigitalInput m_noteDetector;

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
        });// .until(() -> hasGamePiece());
    }

    /*
     * // true is to pass to shooter, false is to recieve from shooter
     * public Command handoff(boolean direction) {
     * State desiredState = new State(0.5 );
     * return this.run(() -> {
     * setState(desiredState);
     * }).until(hasGamePiece());
     * }
     */

    /*
     * public boolean hasGamePiece() {
     * return m_noteDetector.get();
     * }
     */
}