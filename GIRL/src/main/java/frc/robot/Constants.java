// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftMotorPort = 3;
    public static final int kRearLeftMotorPort = 4;
    public static final int kFrontRightMotorPort = 1;
    public static final int kRearRightMotorPort = 2;

    public static final boolean kFrontLeftEncoderReversed = true;
    public static final boolean kRearLeftEncoderReversed = true;
    public static final boolean kFrontRightEncoderReversed = false;
    public static final boolean kRearRightEncoderReversed = false;

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7;
    // Distance between centers of front and back wheels on robot

    public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.
    public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(1, 0.8, 0.15);

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPFrontLeftVel = 0.5;
    public static final double kPRearLeftVel = 0.5;
    public static final double kPFrontRightVel = 0.5;
    public static final double kPRearRightVel = 0.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriveSlowButton = 1; // TODO: Button to half the speed of the drive train.
    public static final int kDriverFireButton = 2; // TODO: Button for Driver to fire shots.
    public static final int kOperatorControllerPort = 0;
    public static final int kOperatorFireButton = 2; // TODO: Button for Operator to fire shots.
    public static final int kOperatorAmpButton = 3; // TODO: Button for Operator to go to amp position.
    public static final int kOperatorSourceButton = 4; // TODO: Button for Operator to go to source position.
    public static final int kOperatorIntakeButton = 5; // TODO: Button for Operator to run intake.
  }

  public static final class ArmConstants {
    public static final int kShoulderMotorPort = 7;
    public static final int kElbowMotorPort = 6;

    public static final double kUpperArmLength = 0.5;
    // Distance between centers of upper arm and forearm pivots on robot
    public static final double kForeArmLength = 0.7;
    // Distance between centers of forearm pivot and shooter launch point on robot

    public static final double kShoulderGearRatio = 450;
    public static final double kElbowGearRatio = 115.56;
    public static final double kShoulderEncoderDistancePerRotation = 360 / kShoulderGearRatio;

    public static final double kElbowEncoderDistancePerRotation = 360 / kElbowGearRatio;

    public static final DCMotor kShoulderMotor = DCMotor.getNEO(1).withReduction(kShoulderGearRatio);
    public static final DCMotor kElbowMotor = DCMotor.getNEO(1).withReduction(kElbowGearRatio);
    // gearbox with 90% efficiency.
    public static final double kShoulderMaxAttainableSpeed = 45; // degrees per second
    public static final double kElbowMaxAttainableSpeed = 45; // degrees per second
    // time to top speed: 1.5 seconds.
    public static final double kShoulderMaxAcceleration = kShoulderMaxAttainableSpeed / 1.5; // degrees per second
    public static final double kElbowMaxAcceleration = kElbowMaxAttainableSpeed / 1.5; // degrees per second^2
    public static final double kAllowedErr = 1; // degrees

    public static final double kElbowResetAngle = 0.0; // degrees. TODO: Set proper resting angle.
    public static final double kShoulderResetAngle = 0.0; // degrees. TODO: Set proper resting angle.
    // public static final double kHandoffShoulderAngle = 45.0; // degrees.
    // public static final double kHandoffElbowAngle = 45.0; // degrees.
    public static final double kSourceShoulderAngle = 45.0; // degrees. TODO: Set proper source angle.
    public static final double kSourceElbowAngle = 60.0; // degrees. TODO: Set proper source angle.
    public static final double kAmpShoulderAngle = 45.0; // degrees. TODO: Set proper Amp angle.
    public static final double kAmpElbowAngle = 135.0; // degrees. TODO: Set proper Amp angle.
    // public static final double kClimbShoulderAngle = 45.0; // degrees.
    // public static final double kClimbElbowAngle = 45.0; // degrees.
    // public static final double kPodiumShoulderAngle = 45.0; // degrees.
    // public static final double kPodiumElbowAngle = 45.0; // degrees.
    // public static final double kSubwooferShoulderAngle = 45.0; // degrees.
    // public static final double kSubwooferElbowAngle = 45.0; // degrees.

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.
    public static final ArmFeedforward kShoulderFeedforward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
    // ks: Volts, TODO: hold arm straight up, slightly increase voltage until arm moves.
    // kg: Volts, TODO: hold arm horizontal, slightly increase voltage until arm holds itself up. 
    // kv: Volt-seconds per degree,
    // ka: Volt-seconds^2 per degree;

    public static final ArmFeedforward kElbowFeedforward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0); // Volts, Volts,
                                                                                                   // Volt-seconds per
                                                                                                   // degree,
                                                                                                   // Volt-seconds^2 per
                                                                                                   // degree;

    // Example value only - as above, this must be tuned for your drive!
    public static final Gains kPIDShoulder = new Gains(0.0, 0.0, 0.0, 0.0, 0.0); // Volts per degree, Volts per
                                                                                 // degree-seconds, Volt-seconds per
                                                                                 // degree,IGNORE, Degrees.
    public static final Gains kPIDElbow = new Gains(0.0, 0.0, 0.0, 0.0, 0.0); // Volts per degree, Volts per
                                                                              // degree-seconds, Volt-seconds per
                                                                              // degree,IGNORE, Degrees.
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
