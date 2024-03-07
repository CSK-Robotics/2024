package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    public enum Goal {
        HOOKHIGH, UNHOOKHIGH, HOOKLOWUNLOCKED, UNHOOKLOWLOCKED, CLIMB, UNCLIMB, STOW
    }

    public class State {
        public Goal goal;
        public boolean isLocked;
        public double elevatorPosition;    
    }

    private final CANSparkFlex m_elevator = new CANSparkFlex(11, CANSparkMax.MotorType.kBrushless);
    private final DoubleSolenoid m_lock = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

    public Command stow() {
        return null;
    }

    public Command deploy() {
        return null;
    }

    public Command climb() {
        return null;
    }

    public Command unclimb() {
        return null;
    }
}
