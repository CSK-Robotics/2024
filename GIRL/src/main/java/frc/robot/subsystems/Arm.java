package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    public enum Goal {
        HANDOFF, SPEAKER, AMP, CLIMB, TRAP, SOURCE
    }

    public class State {
        public Goal goal;
        public double shoulderAngle;
        public double elbowAngle;

        public State(Goal goal, double shoulderAngle, double elbowAngle) {
            this.goal = goal;
            this.shoulderAngle = shoulderAngle;
            this.elbowAngle = elbowAngle;
        }

        public State(double height, double angle) {
            this.goal = Goal.SPEAKER;
            this.shoulderAngle = height;
            this.elbowAngle = angle;
        }
    }

    public enum Operation {
        TUCKING, HOLDING, SHOULDER, ELBOW, SYNCED
    }

    private State setpoint, previous;
    private Operation mode;

    private final CANSparkMax m_shoulder = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax m_elbow = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);

    private final AbsoluteEncoder m_shoulderEncoder = m_shoulder.getAbsoluteEncoder(Type.kDutyCycle);
    private final AbsoluteEncoder m_elbowEncoder = m_elbow.getAbsoluteEncoder(Type.kDutyCycle);

    private final SparkPIDController m_shoulderController = m_shoulder.getPIDController();
    private final SparkPIDController m_elbowController = m_elbow.getPIDController();

    private final TrapezoidProfile m_shoulderProfile = new TrapezoidProfile(
            new Constraints(ArmConstants.kShoulderMaxAttainableSpeed, ArmConstants.kShoulderMaxAcceleration));
    private final TrapezoidProfile m_elbowProfile = new TrapezoidProfile(
            new Constraints(ArmConstants.kElbowMaxAttainableSpeed, ArmConstants.kElbowMaxAcceleration));

    @Override
    public void periodic() {
        State currentState = getState();
        updateOperationMode(currentState);
        switch (this.mode) {
            case SHOULDER:
                driveShoulder(this.setpoint.shoulderAngle);
                holdElbow(ArmConstants.kElbowTuckAngle);
                break;
            case ELBOW:
                driveElbow(this.setpoint.elbowAngle);
                holdShoulder(this.setpoint.shoulderAngle);
                break;
            case TUCKING:
                tuck(currentState);
                break;
            case HOLDING:
            default:
                holdShoulder(this.setpoint.shoulderAngle);
                holdElbow(this.setpoint.elbowAngle);
                break;
        }
        this.previous = currentState;
    }

    private void updateOperationMode(State currentState) {
        if (!isShoulderAtTarget(currentState.shoulderAngle)) {
            if (isTucked(currentState.elbowAngle)) {
                this.mode = Operation.SHOULDER;
                return;
            }
            this.mode = Operation.TUCKING;
            return;
        } else if (!isElbowAtTarget(currentState.elbowAngle)) {
            this.mode = Operation.ELBOW;
            return;
        }
        this.mode = Operation.HOLDING;
    }

    private void tuck(State currentState) {
        if (isTucked(currentState.elbowAngle)) {
            currentState.shoulderAngle = this.previous.shoulderAngle;
            holdShoulder(currentState.shoulderAngle);
            holdElbow(ArmConstants.kElbowTuckAngle);
            return;
        }
        if (!isTuckable(currentState.shoulderAngle)) {
            if (currentState.shoulderAngle < ArmConstants.kShoulderTuckMinimum) {
                driveShoulder(ArmConstants.kShoulderTuckMinimum);
                currentState.elbowAngle = this.previous.elbowAngle;
                holdElbow(currentState.elbowAngle);
                return;
            }
            driveShoulder(ArmConstants.kShoulderTuckMaximum);
            currentState.elbowAngle = this.previous.elbowAngle;
            holdElbow(currentState.elbowAngle);
            return;
        }
        driveElbow(ArmConstants.kElbowTuckAngle);
        currentState.shoulderAngle = this.previous.shoulderAngle;
        holdShoulder(currentState.shoulderAngle);
    }

    private void holdShoulder(double currentAngle) {
        m_shoulderController.setReference(currentAngle, ControlType.kSmartMotion, 0,
                ArmConstants.kShoulderFeedforward.calculate(this.setpoint.shoulderAngle, 0.0));
    }

    private void holdElbow(double currentAngle) {
        m_elbowController.setReference(currentAngle, ControlType.kSmartMotion, 0,
                ArmConstants.kElbowFeedforward.calculate(this.setpoint.elbowAngle, 0.0));
    }

    private void driveElbow(double currentAngle) {
        TrapezoidProfile.State goal = m_elbowProfile.calculate(0.02,
                new TrapezoidProfile.State(currentAngle, this.m_elbowEncoder.getVelocity()),
                new TrapezoidProfile.State(this.setpoint.elbowAngle, 0.0));
        m_elbowController.setReference(goal.velocity, ControlType.kVelocity, 0,
                ArmConstants.kElbowFeedforward.calculate(goal.position, goal.velocity));
    }

    private void driveShoulder(double currentAngle) {
        TrapezoidProfile.State goal = m_shoulderProfile.calculate(0.02,
                new TrapezoidProfile.State(currentAngle, this.m_shoulderEncoder.getVelocity()),
                new TrapezoidProfile.State(this.setpoint.shoulderAngle, 0.0));
        m_shoulderController.setReference(goal.velocity, ControlType.kVelocity, 0,
                ArmConstants.kShoulderFeedforward.calculate(goal.position, goal.velocity));
    }

    private void setState(State desiredState) {
        if (isLegal(desiredState)) {
            this.setpoint = desiredState;
        }
    }

    public Command handoff() {
        return this.runEnd(() -> {
            setState(new State(Goal.HANDOFF, ArmConstants.kHandoffShoulderAngle,
                    ArmConstants.kHandoffElbowAngle));
        }, () -> {this.mode = Operation.HOLDING;}).until(this::atTarget);
    }

    public Command source() {
        return this.runEnd(() -> {
            setState(new State(Goal.SOURCE, ArmConstants.kSourceShoulderAngle, ArmConstants.kSourceElbowAngle));
        }, () -> {this.mode = Operation.HOLDING;}).until(this::atTarget);
    }

    public Command amp() {
        return this.runEnd(() -> {
            setState(new State(Goal.AMP, ArmConstants.kAmpShoulderAngle, ArmConstants.kAmpElbowAngle));
        }, () -> {this.mode = Operation.HOLDING;}).until(this::atTarget);
    }

    public Command shoot(double height, double angle, Goal goal) {
        return this.runEnd(() -> {
            setState(new State(height, angle));
        }, () -> {this.mode = Operation.HOLDING;}).until(this::atTarget);
    }

    public Command climb() {
        return this.runEnd(() -> {
            setState(new State(Goal.CLIMB, ArmConstants.kClimbShoulderAngle, ArmConstants.kClimbElbowAngle));
        }, () -> {this.mode = Operation.HOLDING;}).until(this::atTarget);
    }

    public Command trap() {
        return null;
    }

    public State getState() {
        return new State(this.setpoint.goal, this.m_shoulderEncoder.getPosition(), this.m_elbowEncoder.getPosition());
    }

    public boolean atTarget() {
        State currentState = getState();
        return isElbowAtTarget(currentState.elbowAngle) && isShoulderAtTarget(currentState.shoulderAngle)
                && mode == Operation.HOLDING;
    }

    private boolean isShoulderAtTarget(double currentShoulder) {
        return Math.abs(this.setpoint.shoulderAngle - currentShoulder) < ArmConstants.kAllowedErr;
    }

    private boolean isElbowAtTarget(double currentElbow) {
        return Math.abs(this.setpoint.elbowAngle - currentElbow) < ArmConstants.kAllowedErr;
    }

    private boolean isTucked(double currentElbow) {
        return Math.abs(ArmConstants.kElbowTuckAngle - currentElbow) < ArmConstants.kAllowedErr;
    }

    private boolean isTuckable(double currentShoulder) {
        return currentShoulder > ArmConstants.kShoulderTuckMinimum
                && currentShoulder < ArmConstants.kShoulderTuckMaximum;
    }

    private boolean isLegal(State desiredState) {
        return true;
    }
}
