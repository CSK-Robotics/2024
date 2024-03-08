package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    /*
     * public enum Goal {
     * HANDOFF, SPEAKER, AMP, CLIMB, TRAP, SOURCE
     * }
     */

    public class State {
        // public Goal goal;
        public double shoulderAngle;
        public double elbowAngle;

        public State(double shoulderAngle, double elbowAngle) {
            // this.goal = goal;
            this.shoulderAngle = shoulderAngle;
            this.elbowAngle = elbowAngle;
        }
        /*
         * public State(double height, double angle) {
         * this.goal = Goal.SPEAKER;
         * this.shoulderAngle = height;
         * this.elbowAngle = angle;
         * }
         */
    }

    /*
     * public enum Operation {
     * TUCKING, HOLDING, SHOULDER, ELBOW, SYNCED
     * }
     */

    public Arm(AbsoluteEncoder shoulderToChassis, AbsoluteEncoder shoulderToArm) {
        m_shoulderToChassisEncoder = shoulderToChassis;
        m_shoulderToArmEncoder = shoulderToArm;
        m_elbowEncoder.setPositionConversionFactor(360);
        m_elbowEncoder.setVelocityConversionFactor(360 / 60);

        m_shoulderToArmEncoder.setPositionConversionFactor(360);
        m_shoulderToArmEncoder.setVelocityConversionFactor(360 / 60);

        m_shoulderToChassisEncoder.setPositionConversionFactor(360);
        m_shoulderToChassisEncoder.setVelocityConversionFactor(360 / 60);


        m_shoulderEncoder.setPositionConversionFactor(ArmConstants.kShoulderEncoderDistancePerRotation);
        m_shoulderEncoder.setVelocityConversionFactor(ArmConstants.kShoulderEncoderDistancePerRotation / 60);
        setpoint = new State(getShoulderAngle(), getElbowAngle());

        this.setSubsystem("Arm");
        /*
         * Command shoulderFD = shoulderSysIdDynamic(Direction.kForward),
         * shoulderRD = shoulderSysIdDynamic(Direction.kReverse),
         * shoulderFQ = shoulderSysIdQuasistatic(Direction.kForward),
         * shoulderRQ = shoulderSysIdQuasistatic(Direction.kReverse);
         * shoulderFD.setName("ShoulderForwardDynamic");
         * shoulderRD.setName("ShoulderReverseDynamic");
         * shoulderFQ.setName("ShoulderForwardQuasistatic");
         * shoulderRQ.setName("ShoulderReverseQuasistatic");
         * SmartDashboard.putData(shoulderFD);
         * SmartDashboard.putData(shoulderRD);
         * SmartDashboard.putData(shoulderFQ);
         * SmartDashboard.putData(shoulderRQ);
         * 
         * Command elbowFD = elbowSysIdDynamic(Direction.kForward),
         * elbowRD = elbowSysIdDynamic(Direction.kReverse),
         * elbowFQ = elbowSysIdQuasistatic(Direction.kForward),
         * elbowRQ = elbowSysIdQuasistatic(Direction.kReverse);
         * elbowFD.setName("elbowForwardDynamic");
         * elbowRD.setName("elbowReverseDynamic");
         * elbowFQ.setName("elbowForwardQuasistatic");
         * elbowRQ.setName("elbowReverseQuasistatic");
         * SmartDashboard.putData(elbowFD);
         * SmartDashboard.putData(elbowRD);
         * SmartDashboard.putData(elbowFQ);
         * SmartDashboard.putData(elbowRQ);
         */

        m_elbow.stopMotor();
        m_shoulder.stopMotor();

        SmartDashboard.putNumber("Elbow Angle Setter", 0.0);
        SmartDashboard.putNumber("Shoulder Angle Setter", 0.0);
        SmartDashboard.putBoolean("Joint Switch", true); // True for elbow, false for shoulder

        setDefaultCommand(zero());

        SmartDashboard.putData(tuningVoltage());
        SmartDashboard.putData(tuningPosition());
    }

    private State setpoint;

    private final CANSparkMax m_shoulder = new CANSparkMax(ArmConstants.kShoulderMotorPort,
            CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax m_elbow = new CANSparkMax(ArmConstants.kElbowMotorPort, CANSparkMax.MotorType.kBrushless);

    private final RelativeEncoder m_shoulderEncoder = m_shoulder.getEncoder();
    private final AbsoluteEncoder m_shoulderToArmEncoder;
    private final AbsoluteEncoder m_shoulderToChassisEncoder;
    private final AbsoluteEncoder m_elbowEncoder = m_elbow.getAbsoluteEncoder(Type.kDutyCycle);

    // private final SparkPIDController m_shoulderController =
    // m_shoulder.getPIDController();
    // private final SparkPIDController m_elbowController =
    // m_elbow.getPIDController();

    private final ProfiledPIDController m_shoulderController = new ProfiledPIDController(ArmConstants.kPIDShoulder.kP,
            ArmConstants.kPIDShoulder.kI, ArmConstants.kPIDShoulder.kD,
            new TrapezoidProfile.Constraints(Units.DegreesPerSecond.of(ArmConstants.kShoulderMaxAttainableSpeed),
                    Units.DegreesPerSecond.of(ArmConstants.kShoulderMaxAcceleration).per(Units.Second)));

    private final ProfiledPIDController m_elbowController = new ProfiledPIDController(ArmConstants.kPIDElbow.kP,
            ArmConstants.kPIDElbow.kI, ArmConstants.kPIDElbow.kD,
            new TrapezoidProfile.Constraints(Units.DegreesPerSecond.of(ArmConstants.kElbowMaxAttainableSpeed),
                    Units.DegreesPerSecond.of(ArmConstants.kElbowMaxAcceleration).per(Units.Second)));

    /*
     * private final TrapezoidProfile m_shoulderProfile = new TrapezoidProfile(
     * new Constraints(ArmConstants.kShoulderMaxAttainableSpeed,
     * ArmConstants.kShoulderMaxAcceleration));
     * private final TrapezoidProfile m_elbowProfile = new TrapezoidProfile(
     */

    /*
     * SYSID
     * 
     * private final SysIdRoutine elbowSysIdRoutine = new SysIdRoutine(
     * new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(0.25),
     * Units.Volts.of(4.0), null),
     * new SysIdRoutine.Mechanism(
     * (voltage) -> m_elbowController.setReference(voltage.in(Units.Volts),
     * ControlType.kVoltage),
     * (log) -> {
     * MotorLog motor = log.motor("elbow");
     * motor.angularPosition(Units.Degrees.of(getElbowAngleToGround()));
     * motor.angularVelocity(Units.DegreesPerSecond.of(getElbowVelocity()));
     * motor.voltage(Units.Volts.of(getElbowVoltage()));
     * }, this));
     * 
     * private final SysIdRoutine shoulderSysIdRoutine = new SysIdRoutine(
     * new SysIdRoutine.Config(),
     * new SysIdRoutine.Mechanism(
     * (voltage) -> m_shoulderController.setReference(voltage.in(Units.Volts),
     * ControlType.kVoltage),
     * (log) -> {
     * MotorLog motor = log.motor("shoulder");
     * motor.angularPosition(Units.Degrees.of(getShoulderAngle()));
     * motor.angularVelocity(Units.DegreesPerSecond.of(getShoulderVelocity()));
     * motor.voltage(Units.Volts.of(getShoulderVoltage()));
     * }, this));
     */

    @Override
    public void periodic() {
        /*
         * State currentState = getState();
         * updateOperationMode(currentState);
         * switch (this.mode) {
         * case SHOULDER:
         * driveShoulder(this.setpoint.shoulderAngle);
         * holdElbow(ArmConstants.kElbowTuckAngle);
         * break;
         * case ELBOW:
         * driveElbow(this.setpoint.elbowAngle);
         * holdShoulder(this.setpoint.shoulderAngle);
         * break;
         * case TUCKING:
         * tuck(currentState);
         * break;
         * case HOLDING:
         * default:
         * holdShoulder(this.setpoint.shoulderAngle);
         * holdElbow(this.setpoint.elbowAngle);
         * break;
         * }
         * this.previous = currentState;
         */

        State currentState = getState();

        SmartDashboard.putNumber("Elbow Angle", currentState.elbowAngle);
        SmartDashboard.putNumber("Shoulder Angle", currentState.shoulderAngle);

        if (setpoint != null) {
            setElbowPosition(currentState, setpoint.elbowAngle);
            setShoulderPosition(currentState, setpoint.shoulderAngle);
        } else {
            if (SmartDashboard.getBoolean("JointSwitch", true)) {
                m_elbow.setVoltage(SmartDashboard.getNumber("Elbow Voltage Setter", 0.0));
                setShoulderPosition(currentState, m_shoulderController.getSetpoint().position);
            } else {
                setElbowPosition(currentState, ArmConstants.kElbowResetAngle);
                m_shoulder.setVoltage(SmartDashboard.getNumber("Shoulder Voltage Setter", 0.0));
            }
        }
    }

    /*
     * private void updateOperationMode(State currentState) {
     * if (!isShoulderAtTarget(currentState.shoulderAngle)) {
     * if (isTucked(currentState.elbowAngle)) {
     * this.mode = Operation.SHOULDER;
     * return;
     * }
     * this.mode = Operation.TUCKING;
     * return;
     * } else if (!isElbowAtTarget(currentState.elbowAngle)) {
     * this.mode = Operation.ELBOW;
     * return;
     * }
     * this.mode = Operation.HOLDING;
     * }
     * 
     * private void tuck(State currentState) {
     * if (isTucked(currentState.elbowAngle)) {
     * currentState.shoulderAngle = this.previous.shoulderAngle;
     * holdShoulder(currentState.shoulderAngle);
     * holdElbow(ArmConstants.kElbowTuckAngle);
     * return;
     * }
     * if (!isTuckable(currentState.shoulderAngle)) {
     * if (currentState.shoulderAngle < ArmConstants.kShoulderTuckMinimum) {
     * driveShoulder(ArmConstants.kShoulderTuckMinimum);
     * currentState.elbowAngle = this.previous.elbowAngle;
     * holdElbow(currentState.elbowAngle);
     * return;
     * }
     * driveShoulder(ArmConstants.kShoulderTuckMaximum);
     * currentState.elbowAngle = this.previous.elbowAngle;
     * holdElbow(currentState.elbowAngle);
     * return;
     * }
     * driveElbow(ArmConstants.kElbowTuckAngle);
     * currentState.shoulderAngle = this.previous.shoulderAngle;
     * holdShoulder(currentState.shoulderAngle);
     * }
     */

    private void setElbowPosition(State currentState, double angle) {
        m_elbowController.setGoal(angle);
        m_elbow.setVoltage(
                m_elbowController.calculate(currentState.elbowAngle) + ArmConstants.kElbowFeedforward.calculate(
                        Rotation2d.fromDegrees(getElbowAngleToGround(currentState)).getRadians(),
                        m_elbowController.getSetpoint().velocity));
    }

    private void setShoulderPosition(State currentState, double angle) {
        m_shoulderController.setGoal(angle);
        m_shoulder.setVoltage(m_shoulderController.calculate(currentState.shoulderAngle)
                + ArmConstants.kShoulderFeedforward.calculate(
                        Rotation2d.fromDegrees(currentState.shoulderAngle).getRadians(),
                        m_shoulderController.getSetpoint().velocity));
    }

    private void setState(State desiredState) {
        this.setpoint = desiredState;
    }

    public Command zero() {
        return this.run(() -> {
            setState(new State(m_shoulderController.getSetpoint().position, ArmConstants.kElbowResetAngle));
        }).andThen(this.run(() -> {
            setState(new State(ArmConstants.kShoulderResetAngle, ArmConstants.kElbowResetAngle));
        }));
    }

    public Command tuningPosition() {
        return this.run(() -> {
            setState(new State(SmartDashboard.getNumber("Elbow Angle Setter", 0.0),
                    SmartDashboard.getNumber("Shoulder Angle Setter", 0.0)));
        });
    }

    public Command tuningVoltage() {
        return this.run(() -> {
            setState(null);
        });
    }
    /*
     * public Command handoff() {
     * return this.run(() -> {
     * setState(new State(ArmConstants.kHandoffShoulderAngle,
     * ArmConstants.kHandoffElbowAngle));
     * });
     * }
     */

    public Command source() {
        return this.run(() -> {
            setState(new State(ArmConstants.kSourceShoulderAngle, ArmConstants.kSourceElbowAngle));
        });
    }

    public Command amp() {
        return this.run(() -> {
            setState(new State(ArmConstants.kAmpShoulderAngle, ArmConstants.kAmpElbowAngle));
        });
    }
    /*
     * public Command podium() {
     * return this.run(() -> {
     * setState(new State(ArmConstants.kPodiumShoulderAngle,
     * ArmConstants.kPodiumElbowAngle));
     * });
     * }
     */

    /*
     * public Command subwoofer() {
     * return this.run(() -> {
     * setState(new State(ArmConstants.kSubwooferShoulderAngle,
     * ArmConstants.kSubwooferElbowAngle));
     * });
     * }
     */

    /*
     * public Command shoot(double height, double angle) {
     * return this.run(() -> {
     * setState(new State(height, angle));
     * });
     * }
     */

    /*
     * public Command climb() {
     * return this.runEnd(() -> {
     * setState(new State(Goal.CLIMB, ArmConstants.kClimbShoulderAngle,
     * ArmConstants.kClimbElbowAngle));
     * }, () -> {
     * this.mode = Operation.HOLDING;
     * }).until(this::atTarget);
     * }
     * 
     * public Command trap() {
     * return null;
     * }
     */

    public State getState() {
        return new State(getShoulderAngle(), getElbowAngle());
    }

    public boolean atTarget() {
        State currentState = getState();
        return isElbowAtTarget(currentState.elbowAngle) && isShoulderAtTarget(currentState.shoulderAngle);
    }

    private boolean isShoulderAtTarget(double currentShoulder) {
        return Math.abs(this.setpoint.shoulderAngle - currentShoulder) < ArmConstants.kAllowedErr;
    }

    private boolean isElbowAtTarget(double currentElbow) {
        return Math.abs(this.setpoint.elbowAngle - currentElbow) < ArmConstants.kAllowedErr;
    }

    private double getShoulderAngle() {
        return Rotation2d.fromDegrees(m_shoulderToChassisEncoder.getPosition())
                .minus(Rotation2d.fromDegrees(m_shoulderToArmEncoder.getPosition())).getDegrees();
    }

    private double getElbowAngle() {
        return m_elbowEncoder.getPosition();
    }

    private double getElbowAngleToGround(State state) {
        return Rotation2d.fromDegrees(state.elbowAngle).rotateBy(Rotation2d.fromDegrees(state.shoulderAngle))
                .getDegrees();
    }

    /*
     * private double getShoulderVelocity() {
     * return m_shoulderEncoder.getVelocity();
     * }
     * 
     * private double getElbowVelocity() {
     * return m_elbowEncoder.getVelocity();
     * }
     * 
     * private double getShoulderVoltage() {
     * return m_shoulder.getBusVoltage() * m_shoulder.getAppliedOutput();
     * }
     * 
     * private double getElbowVoltage() {
     * return m_elbow.getBusVoltage() * m_elbow.getAppliedOutput();
     * }
     */

    /*
     * 
     * SYSID Commands
     * Returns a command that will excute a quasistatic command in the given
     * direction
     * 
     * public Command elbowSysIdQuasistatic(SysIdRoutine.Direction direction) {
     * return elbowSysIdRoutine.quasistatic(direction);
     * }
     * 
     * /**
     * Returns a command that will excute a dynamic command in the given direction
     * 
     * public Command elbowSysIdDynamic(SysIdRoutine.Direction direction) {
     * return elbowSysIdRoutine.dynamic(direction);
     * }
     * 
     * /**
     * Returns a command that will excute a quasistatic command in the given
     * direction
     * 
     * public Command shoulderSysIdQuasistatic(SysIdRoutine.Direction direction) {
     * return shoulderSysIdRoutine.quasistatic(direction);
     * }
     * 
     * /**
     * Returns a command that will excute a dynamic command in the given direction
     * 
     * public Command shoulderSysIdDynamic(SysIdRoutine.Direction direction) {
     * return shoulderSysIdRoutine.dynamic(direction);
     * }
     */
}