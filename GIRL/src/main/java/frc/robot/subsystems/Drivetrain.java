// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Drivetrain extends SubsystemBase {
  private final CANSparkMax m_frontLeft = new CANSparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rearLeft = new CANSparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_frontRight = new CANSparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rearRight = new CANSparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushless);

  private final MecanumDrive m_drive = new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set,
      m_rearRight::set);

  // The front-left-side drive encoder
  private final RelativeEncoder m_frontLeftEncoder = m_frontLeft.getEncoder();
  // and PID Controller
  private final SparkPIDController m_frontLeftController = m_frontLeft.getPIDController();

  // The rear-left-side drive encoder
  private final RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder();
  // and PID Controller
  private final SparkPIDController m_rearLeftController = m_rearLeft.getPIDController();

  // The front-right--side drive encoder
  private final RelativeEncoder m_frontRightEncoder = m_frontRight.getEncoder();
  // and PID Controller
  private final SparkPIDController m_frontRightController = m_frontRight.getPIDController();

  // The rear-right-side drive encoder
  private final RelativeEncoder m_rearRightEncoder = m_rearRight.getEncoder();
  // and PID Controller
  private final SparkPIDController m_rearRightController = m_rearRight.getPIDController();

  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  MecanumDrivePoseEstimator m_odometry = new MecanumDrivePoseEstimator(DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(), new MecanumDriveWheelPositions(), new Pose2d());

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    SendableRegistry.addChild(m_drive, m_frontLeft);
    SendableRegistry.addChild(m_drive, m_rearLeft);
    SendableRegistry.addChild(m_drive, m_frontRight);
    SendableRegistry.addChild(m_drive, m_rearRight);

    // Sets the distance per pulse for the encoders
    m_frontLeftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerRotation);
    m_rearLeftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerRotation);
    m_frontRightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerRotation);
    m_rearRightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerRotation);
    m_frontLeftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRotation);
    m_rearLeftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRotation);
    m_frontRightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRotation);
    m_rearRightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRotation);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);

    m_frontLeftController.setP(DriveConstants.kPIDFrontLeftVel.kP);
    m_frontLeftController.setI(DriveConstants.kPIDFrontLeftVel.kI);
    m_frontLeftController.setD(DriveConstants.kPIDFrontLeftVel.kD);
    m_frontLeftController.setFF(DriveConstants.kPIDFrontLeftVel.kFF);
    m_frontLeftController.setIZone(DriveConstants.kPIDFrontLeftVel.kIz);
    m_frontLeftController.setOutputRange(-DriveConstants.kMaxAttainableSpeed, DriveConstants.kMaxAttainableSpeed);

    m_frontRightController.setP(DriveConstants.kPIDFrontRightVel.kP);
    m_frontRightController.setI(DriveConstants.kPIDFrontRightVel.kI);
    m_frontRightController.setD(DriveConstants.kPIDFrontRightVel.kD);
    m_frontRightController.setFF(DriveConstants.kPIDFrontRightVel.kFF);
    m_frontRightController.setIZone(DriveConstants.kPIDFrontRightVel.kIz);
    m_frontRightController.setOutputRange(-DriveConstants.kMaxAttainableSpeed, DriveConstants.kMaxAttainableSpeed);

    m_rearLeftController.setP(DriveConstants.kPIDRearLeftVel.kP);
    m_rearLeftController.setI(DriveConstants.kPIDRearLeftVel.kI);
    m_rearLeftController.setD(DriveConstants.kPIDRearLeftVel.kD);
    m_rearLeftController.setFF(DriveConstants.kPIDRearLeftVel.kFF);
    m_rearLeftController.setIZone(DriveConstants.kPIDRearLeftVel.kIz);
    m_rearLeftController.setOutputRange(-DriveConstants.kMaxAttainableSpeed, DriveConstants.kMaxAttainableSpeed);

    m_rearRightController.setP(DriveConstants.kPIDRearRightVel.kP);
    m_rearRightController.setI(DriveConstants.kPIDRearRightVel.kI);
    m_rearRightController.setD(DriveConstants.kPIDRearRightVel.kD);
    m_rearRightController.setFF(DriveConstants.kPIDRearRightVel.kFF);
    m_rearRightController.setIZone(DriveConstants.kPIDRearRightVel.kIz);
    m_rearRightController.setOutputRange(-DriveConstants.kMaxAttainableSpeed, DriveConstants.kMaxAttainableSpeed);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), getCurrentWheelDistances());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getCurrentWheelDistances(), pose);
  }

  public void addVisionMeasurement(Pose2d measurement, double timestamp) {
    m_odometry.addVisionMeasurement(measurement, timestamp);
  }

  /**
   * Set the desired speeds for each wheel.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    final double frontLeftFeedforward = DriveConstants.kFeedforward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedforward = DriveConstants.kFeedforward.calculate(speeds.frontRightMetersPerSecond);
    final double rearLeftFeedforward = DriveConstants.kFeedforward.calculate(speeds.rearLeftMetersPerSecond);
    final double rearRightFeedforward = DriveConstants.kFeedforward.calculate(speeds.rearRightMetersPerSecond);

    m_frontLeftController.setReference(speeds.frontLeftMetersPerSecond, ControlType.kVelocity, 0, frontLeftFeedforward);
    m_frontRightController.setReference(speeds.frontRightMetersPerSecond, ControlType.kVelocity, 0,
        frontRightFeedforward);
    m_rearLeftController.setReference(speeds.rearLeftMetersPerSecond, ControlType.kVelocity, 0,
        rearLeftFeedforward);
    m_rearRightController.setReference(speeds.rearRightMetersPerSecond, ControlType.kVelocity, 0, rearRightFeedforward);
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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var mecanumDriveWheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            0.02));
    drive(mecanumDriveWheelSpeeds);
  }

  public void drive(MecanumDriveWheelSpeeds speeds) {
    speeds.desaturate(DriveConstants.kMaxAttainableSpeed);
    setSpeeds(speeds);
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeftEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getVelocity(),
        m_rearLeftEncoder.getVelocity(),
        m_frontRightEncoder.getVelocity(),
        m_rearRightEncoder.getVelocity());
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a
   *         MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getPosition(),
        m_rearLeftEncoder.getPosition(),
        m_frontRightEncoder.getPosition(),
        m_rearRightEncoder.getPosition());
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
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
