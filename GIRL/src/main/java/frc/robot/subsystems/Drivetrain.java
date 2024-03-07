// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Gains;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public final class Drivetrain extends SubsystemBase implements AutoCloseable {
  public final class Wheel {
    private static final class Inputs implements LoggableInputs {
      public double PositionMeters = 0.0;
      public double VelocityMetersPerSec = 0.0;
      public double AppliedVolts = 0.0;
      public double CurrentAmps = 0.0;
      public double TempFahrenheit = 0.0;

      @Override
      public void toLog(LogTable table) {
        table.put("Position", PositionMeters);
        table.put("Velocity", VelocityMetersPerSec);
        table.put("AppliedVoltage", AppliedVolts);
        table.put("OutputCurrent", CurrentAmps);
        table.put("MotorTemperature", TempFahrenheit);
      }

      @Override
      public void fromLog(LogTable table) {
        PositionMeters = table.get("Position").getDouble();
        VelocityMetersPerSec = table.get("Velocity").getDouble();
        AppliedVolts = table.get("AppliedVoltage").getDouble();
        CurrentAmps = table.get("OutputCurrent").getDouble();
        TempFahrenheit = table.get("MotorTemperature").getDouble();
      }
    }

    public Inputs inputs;

    private final CANSparkMax m_motor;
    private final String m_name;
    private final RelativeEncoder m_encoder;
    private final SparkPIDController m_controller;

    public Wheel(int port, String name, MotorType type, Gains gains, boolean invert) {
      inputs = new Inputs();
      m_name = name;
      m_motor = new CANSparkMax(port, type);

      m_motor.restoreFactoryDefaults();
      m_motor.setCANTimeout(250);

      m_motor.setInverted(invert);
      m_encoder = m_motor.getEncoder();
      m_controller = m_motor.getPIDController();

      m_motor.setSmartCurrentLimit(40);
      m_motor.enableVoltageCompensation(12);

      m_encoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerRotation);
      m_encoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRotation);

      m_encoder.setPosition(0);
      m_encoder.setMeasurementPeriod(10);
      m_encoder.setAverageDepth(2);

      gains.configureController(m_controller, DriveConstants.kMaxAttainableSpeed);
      m_controller.setFeedbackDevice(m_encoder);

      m_motor.setCANTimeout(250);
      m_motor.burnFlash();
    }

    public void periodic() {
      updateInputs();
      Logger.processInputs(m_name, inputs);
    }

    public void updateInputs() {
      inputs.VelocityMetersPerSec = m_encoder.getVelocity();
      inputs.PositionMeters = m_encoder.getPosition();
      inputs.AppliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
      inputs.CurrentAmps = m_motor.getOutputCurrent();
      inputs.TempFahrenheit = Units.Fahrenheit.convertFrom(m_motor.getMotorTemperature(), Units.Celsius);
    }

    public void setVelocity(double speed) {
      final double feedforward = DriveConstants.kFeedforward.calculate(speed);
      m_controller.setReference(speed, ControlType.kVelocity, 0, feedforward);
    }

    public void setPosition(double position) {
      m_encoder.setPosition(position);
    }

    public void setVoltage(double voltage) {
      m_controller.setReference(voltage, ControlType.kVoltage);
    }

    public void close() {
      m_motor.close();
    }

    public double getVelocity() {
      return inputs.VelocityMetersPerSec;
    }

    public double getPosition() {
      return inputs.PositionMeters;
    }
  }

  public final class Gyro {
    public static final class Inputs implements LoggableInputs {
      //public Rotation3d orientation = new Rotation3d();
      public Rotation2d rate = new Rotation2d();

      @Override
      public void toLog(LogTable table) {
        //table.put("Orientation/Roll", Units.Degrees.convertFrom(orientation.getX(), Units.Radians));
        //table.put("Orientation/Pitch", Units.Degrees.convertFrom(orientation.getY(), Units.Radians));
        //table.put("Orientation/Yaw", Units.Degrees.convertFrom(orientation.getZ(), Units.Radians));
        table.put("Orientation/Yaw/Rate", rate.getDegrees());
      }

      @Override
      public void fromLog(LogTable table) {
       //orientation = new Rotation3d(table.get("Orientation/Roll").getDouble(),
        //    table.get("Orientation/Pitch").getDouble(),
        //    table.get("Orientation/Yaw").getDouble());
        rate = Rotation2d.fromDegrees(table.get("Orientation/Yaw/Rate").getDouble());
      }
    }

    public Inputs inputs;

    // The gyro sensor
    private final AHRS m_ahrs;

    public Gyro() {
      m_ahrs = new AHRS(Port.kUSB1);
    }

    public void periodic() {
      updateInputs();
      Logger.processInputs("NavX", inputs);
    }

    public void updateInputs() {
      //inputs.orientation = getOrientation();
      inputs.rate = getRate();
    }

    public Rotation3d getOrientation() {
      return m_ahrs.getRotation3d();
    }

    public Rotation2d getRate() {
      return Rotation2d.fromDegrees(m_ahrs.getRate());
    }

    public void reset() {
      m_ahrs.reset();
    }
  }

  // IO classes for wheels
  private final Wheel m_frontLeft;
  private final Wheel m_rearLeft;
  private final Wheel m_frontRight;
  private final Wheel m_rearRight;

  private final MecanumDrive m_drive;

  // IO classes for gyro
  private final Gyro m_gyro;

  // Odometry class for tracking robot pose
  MecanumDrivePoseEstimator m_odometry;

  StructPublisher<Pose2d> publisher;

   /** Creates SysId Routine. */
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, null, null,
          (State state) -> Logger.recordOutput("Drive/SysID/State", state.toString())),
      new SysIdRoutine.Mechanism(
          (voltage) -> drive(new MecanumDriveMotorVoltages(voltage.in(Units.Volts), voltage.in(Units.Volts), voltage.in(Units.Volts), voltage.in(Units.Volts))),
          null, this));

  /** Returns a command that will excute a quasistatic command in the given direction  */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_sysIdRoutine.quasistatic(direction);
  }

  /** Returns a command that will excute a dynamic command in the given direction  */
  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return m_sysIdRoutine.dynamic(direction);
  }

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    m_frontLeft = new Wheel(DriveConstants.kFrontLeftMotorPort, "FrontLeftDriveWheel", MotorType.kBrushless,
        DriveConstants.kPIDFrontLeftVel, true);
    m_frontRight = new Wheel(DriveConstants.kFrontRightMotorPort, "FrontRightDriveWheel", MotorType.kBrushless,
        DriveConstants.kPIDFrontRightVel, false);
    m_rearLeft = new Wheel(DriveConstants.kRearLeftMotorPort, "RearLeftDriveWheel", MotorType.kBrushless,
        DriveConstants.kPIDRearLeftVel, true);
    m_rearRight = new Wheel(DriveConstants.kRearRightMotorPort, "RearRightDriveWheel", MotorType.kBrushless,
        DriveConstants.kPIDRearRightVel, false);

    m_gyro = new Gyro();

    m_drive = new MecanumDrive(m_frontLeft::setVelocity, m_rearLeft::setVelocity, m_frontRight::setVelocity,
        m_rearRight::setVelocity);
    m_odometry = new MecanumDrivePoseEstimator(DriveConstants.kDriveKinematics,
        m_gyro.getOrientation().toRotation2d(), new MecanumDriveWheelPositions(), new Pose2d());

    publisher = NetworkTableInstance.getDefault()
        .getStructTopic("Pose", Pose2d.struct).publish();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getOrientation().toRotation2d(), getCurrentWheelDistances());

    m_frontLeft.periodic();
    m_frontRight.periodic();
    m_rearLeft.periodic();
    m_rearRight.periodic();

    m_gyro.periodic();

    publisher.set(m_odometry.getEstimatedPosition());
  }

  public void close() {
    m_frontLeft.close();
    m_frontRight.close();
    m_rearLeft.close();
    m_rearRight.close();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @AutoLogOutput(key = "Drive/Pose")
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getOrientation().toRotation2d(), getCurrentWheelDistances(), pose);
  }

  public void addVisionMeasurement(Pose2d measurement, double timestamp) {
    m_odometry.addVisionMeasurement(measurement, timestamp);
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    m_drive.driveCartesian(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond,
        fieldRelative ? m_gyro.getOrientation().toRotation2d() : new Rotation2d());
  }

  /** Sets the drive MotorControllers to given speeds. */
  public void drive(MecanumDriveWheelSpeeds speeds) {
    speeds.desaturate(DriveConstants.kMaxAttainableSpeed);
    m_frontLeft.setVelocity(speeds.frontLeftMetersPerSecond);
    m_frontRight.setVelocity(speeds.frontRightMetersPerSecond);
    m_rearLeft.setVelocity(speeds.rearLeftMetersPerSecond);
    m_rearRight.setVelocity(speeds.rearRightMetersPerSecond);
  }

  /** Sets the drive MotorControllers to given voltages. */
  public void drive(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.setPosition(0);
    m_rearLeft.setPosition(0);
    m_frontRight.setPosition(0);
    m_rearRight.setPosition(0);
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  @AutoLogOutput(key = "Drive/Wheel/Speed")
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeft.getVelocity(),
        m_rearLeft.getVelocity(),
        m_frontRight.getVelocity(),
        m_rearRight.getVelocity());
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a
   *         MecanumDriveWheelPositions object.
   */
  @AutoLogOutput(key = "Drive/Wheel/Distance")
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeft.getPosition(),
        m_rearLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearRight.getPosition());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  @AutoLogOutput(key = "Drive/Gyro/Heading")
  public double getHeading() {
    return m_gyro.getOrientation().toRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  @AutoLogOutput(key = "Drive/Gyro/Rate")
  public double getTurnRate() {
    return -m_gyro.getRate().getDegrees();
  }
}
