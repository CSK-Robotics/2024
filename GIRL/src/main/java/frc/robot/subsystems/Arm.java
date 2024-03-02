package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.Gains;

import java.util.Optional;

public class Arm extends SubsystemBase {
    public enum Goal {
        HANDOFF, SPEAKER, AMP, CLIMB, TRAP, SOURCE
    }

    public final class Joint {
        private static final class Inputs implements LoggableInputs {
            public double PositionDegrees = 0.0;
            public double VelocityDegreesPerSec = 0.0;
            public double AppliedVolts = 0.0;
            public double CurrentAmps = 0.0;
            public double TempFahrenheit = 0.0;

            @Override
            public void toLog(LogTable table) {
                table.put("Position", PositionDegrees);
                table.put("Velocity", VelocityDegreesPerSec);
                table.put("AppliedVoltage", AppliedVolts);
                table.put("OutputCurrent", CurrentAmps);
                table.put("MotorTemperature", TempFahrenheit);
            }

            @Override
            public void fromLog(LogTable table) {
                PositionDegrees = table.get("Position").getDouble();
                VelocityDegreesPerSec = table.get("Velocity").getDouble();
                AppliedVolts = table.get("AppliedVoltage").getDouble();
                CurrentAmps = table.get("OutputCurrent").getDouble();
                TempFahrenheit = table.get("MotorTemperature").getDouble();
            }
        }

        public Inputs inputs;

        private final CANSparkMax m_motor;
        private final String m_name;
        private final RelativeEncoder m_encoder;
        private final AbsoluteEncoder m_encoders[];
        private final SparkPIDController m_controller;

        private final ArmFeedforward m_feedforward;
        private final TrapezoidProfile m_profile;

        public Joint(int port, String name, MotorType type, Gains gains, boolean invert,
                Optional<AbsoluteEncoder> encoder, double maxSpeed, double maxAccel, ArmFeedforward feedforward) {
            m_name = name;
            m_motor = new CANSparkMax(port, type);
            m_feedforward = feedforward;

            m_motor.restoreFactoryDefaults();
            m_motor.setCANTimeout(250);

            m_motor.setInverted(invert);
            m_encoder = m_motor.getEncoder();
            m_encoders = encoder.isPresent()
                    ? new AbsoluteEncoder[] { m_motor.getAbsoluteEncoder(Type.kDutyCycle), encoder.get() }
                    : new AbsoluteEncoder[] { m_motor.getAbsoluteEncoder(Type.kDutyCycle) };
            m_controller = m_motor.getPIDController();

            m_motor.setSmartCurrentLimit(30);
            m_motor.enableVoltageCompensation(12);

            m_encoders[0].setAverageDepth(2);

            gains.configureController(m_controller, maxSpeed);
            m_controller.setFeedbackDevice(m_encoder);

            m_motor.setCANTimeout(250);
            m_motor.burnFlash();

            m_profile = new TrapezoidProfile(
                    new Constraints(maxSpeed, maxAccel));
        }

        public void periodic() {
            updateInputs();
            Logger.processInputs(m_name, inputs);
        }

        public void updateInputs() {
            inputs.VelocityDegreesPerSec = getRate();
            inputs.PositionDegrees = getAngle();
            inputs.AppliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
            inputs.CurrentAmps = m_motor.getOutputCurrent();
            inputs.TempFahrenheit = Units.Fahrenheit.convertFrom(m_motor.getMotorTemperature(), Units.Celsius);
        }

        public void driveToPosition(double setpoint) {
            TrapezoidProfile.State goal = m_profile.calculate(0.02,
                    new TrapezoidProfile.State(getAngle(), getVelocity()),
                    new TrapezoidProfile.State(setpoint, 0.0));
            m_controller.setReference(goal.velocity, ControlType.kVelocity, 0,
                    ArmConstants.kElbowFeedforward.calculate(goal.position, goal.velocity));
        }

        public void holdPosition(double position) {
            m_controller.setReference(position, ControlType.kPosition, 0,
                    m_feedforward.calculate(edu.wpi.first.math.util.Units.degreesToRadians(position), 0.0));
        }

        public void setVoltage(double voltage) {
            m_controller.setReference(voltage, ControlType.kVoltage);
        }

        public void close() {
            m_motor.close();
        }

        public double getVelocity() {
            return inputs.VelocityDegreesPerSec;
        }

        public double getPosition() {
            return inputs.PositionDegrees;
        }

        private double getAngle() {
            return m_encoders.length > 1
                    ? Rotation2d.fromDegrees(m_encoders[0].getPosition())
                            .minus(Rotation2d.fromDegrees(m_encoders[1].getPosition())).getDegrees()
                    : m_encoders[0].getPosition();
        }

        private double getRate() {
            return m_encoders.length > 1
                    ? Rotation2d.fromDegrees(m_encoders[0].getVelocity())
                            .minus(Rotation2d.fromDegrees(m_encoders[1].getVelocity())).getDegrees()
                    : m_encoders[0].getVelocity();
        }
    }

    public class State {
        private final String kID;

        @AutoLogOutput (key = "State/{kID}/Goal")
        public Goal goal;

        @AutoLogOutput (key = "State/{kID}/Shoulder/Angle")
        public double shoulderAngle;

        @AutoLogOutput (key = "State/{kID}/Elbow/Angle")
        public double elbowAngle;

        public State(String name) {
            kID = name;
        }
    }

    public enum Operation {
        TUCKING, HOLDING, SHOULDER, ELBOW, SYNCED
    }

    private State setpoint = new State("Setpoint");
    private State previous = new State("Previous");
    private Operation mode;

    private final Joint m_shoulder;
    private final Joint m_elbow;

    public Arm(AbsoluteEncoder encoder) {
        m_shoulder = new Joint(ArmConstants.kShoulderMotorPort, "ArmShoulderJoint", MotorType.kBrushless,
                ArmConstants.kPIDShoulder, false, Optional.of(encoder), ArmConstants.kShoulderMaxAttainableSpeed,
                ArmConstants.kShoulderMaxAcceleration, ArmConstants.kShoulderFeedforward);

        m_elbow = new Joint(ArmConstants.kElbowMotorPort, "ArmElbowJoint", MotorType.kBrushless, ArmConstants.kPIDElbow,
                false, Optional.empty(), ArmConstants.kElbowMaxAttainableSpeed, ArmConstants.kElbowMaxAcceleration,
                ArmConstants.kElbowFeedforward);
    }

    @Override
    public void periodic() {
        State currentState = getState();
        updateOperationMode(currentState);
        switch (this.mode) {
            case SHOULDER:
                m_shoulder.driveToPosition(this.setpoint.shoulderAngle);
                m_elbow.holdPosition(ArmConstants.kElbowTuckAngle);
                break;
            case ELBOW:
                m_elbow.driveToPosition(this.setpoint.elbowAngle);
                m_shoulder.holdPosition(this.setpoint.shoulderAngle);
                break;
            case TUCKING:
                tuck(currentState);
                break;
            case HOLDING:
            default:
                m_shoulder.holdPosition(this.setpoint.shoulderAngle);
                m_elbow.holdPosition(this.setpoint.elbowAngle);
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
            m_shoulder.holdPosition(currentState.shoulderAngle);
            m_elbow.holdPosition(ArmConstants.kElbowTuckAngle);
            return;
        }
        if (!isTuckable(currentState.shoulderAngle)) {
            if (currentState.shoulderAngle < ArmConstants.kShoulderTuckMinimum) {
                m_shoulder.driveToPosition(ArmConstants.kShoulderTuckMinimum);
                currentState.elbowAngle = this.previous.elbowAngle;
                m_elbow.holdPosition(currentState.elbowAngle);
                return;
            }
            m_shoulder.driveToPosition(ArmConstants.kShoulderTuckMaximum);
            currentState.elbowAngle = this.previous.elbowAngle;
            m_elbow.holdPosition(currentState.elbowAngle);
            return;
        }
        m_elbow.driveToPosition(ArmConstants.kElbowTuckAngle);
        currentState.shoulderAngle = this.previous.shoulderAngle;
        m_shoulder.holdPosition(currentState.shoulderAngle);
    }

    private void setState(Goal goal, double shoulder, double elbow) {
        setpoint.goal = goal;
        setpoint.shoulderAngle = shoulder;
        setpoint.elbowAngle = elbow;
    }

    public Command handoff() {
        return this.runEnd(() -> {
            setState(Goal.HANDOFF, ArmConstants.kHandoffShoulderAngle,
                    ArmConstants.kHandoffElbowAngle);
        }, () -> {
            this.mode = Operation.HOLDING;
        }).until(this::atTarget);
    }

    public Command source() {
        return this.runEnd(() -> {
            setState(Goal.SOURCE, ArmConstants.kSourceShoulderAngle, ArmConstants.kSourceElbowAngle);
        }, () -> {
            this.mode = Operation.HOLDING;
        }).until(this::atTarget);
    }

    public Command amp() {
        return this.runEnd(() -> {
            setState(Goal.AMP, ArmConstants.kAmpShoulderAngle, ArmConstants.kAmpElbowAngle);
        }, () -> {
            this.mode = Operation.HOLDING;
        }).until(this::atTarget);
    }

    public Command shoot(double height, double angle, Goal goal) {
        // TODO: calculate state from height and angle
        double elbow = 0.0;
        double shoulder = 0.0;
        return this.runEnd(() -> {
            setState(goal, shoulder, elbow);
        }, () -> {
            this.mode = Operation.HOLDING;
        }).until(this::atTarget);
    }

    public Command climb() {
        return this.runEnd(() -> {
            setState(Goal.CLIMB, ArmConstants.kClimbShoulderAngle, ArmConstants.kClimbElbowAngle);
        }, () -> {
            this.mode = Operation.HOLDING;
        }).until(this::atTarget);
    }

    public Command trap() {
        return null;
    }

    public State getState() {
        return new State(this.setpoint.goal, this.m_shoulder.getPosition(), this.m_elbow.getPosition());
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
}
