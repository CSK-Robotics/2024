// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
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
    public static final int kFrontLeftMotorPort = 1;
    public static final int kRearLeftMotorPort = 2;
    public static final int kFrontRightMotorPort = 3;
    public static final int kRearRightMotorPort = 4;

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
    public static final double kGearRatio = (14 / 70) * (24 / 15);
    public static final double kEncoderDistancePerRotation = (kWheelDiameterMeters * Math.PI) / kGearRatio;

    public static final DCMotor kMotor = DCMotor.getNEO(1).withReduction(kGearRatio);
    // gearbox with 90% efficiency.
    public static final double kMaxAttainableSpeed = kMotor.freeSpeedRadPerSec * kWheelDiameterMeters * 0.90;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.
    public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(1, 0.8, 0.15);

    // Example value only - as above, this must be tuned for your drive!
    public static final Gains kPIDFrontLeftVel = new Gains(0.5, 0.0, 0.0, 0.0, 0.0);
    public static final Gains kPIDRearLeftVel = new Gains(0.5, 0.0, 0.0, 0.0, 0.0);
    public static final Gains kPIDFrontRightVel = new Gains(0.5, 0.0, 0.0, 0.0, 0.0);
    public static final Gains kPIDRearRightVel = new Gains(0.5, 0.0, 0.0, 0.0, 0.0);

    public static final Gains kPIDX = new Gains(0.0, 0.0, 0.0, 0.0, 0.0);
    public static final Gains kPIDY = new Gains(0.0, 0.0, 0.0, 0.0, 0.0);
    public static final Gains kPIDT = new Gains(0.0, 0.0, 0.0, 0.0, 0.0);
  }

  public static final class ArmConstants {
    public static final int kShoulderMotorPort = 5;
    public static final int kElbowMotorPort = 6;

    public static final double kUpperArmLength = 0.5;
    // Distance between centers of upper arm and forearm pivots on robot
    public static final double kForeArmLength = 0.7;
    // Distance between centers of forearm pivot and shooter launch point on robot

    public static final int kEncoderCPR = 1024;
    public static final double kShoulderGearRatio = (14 / 70) * (24 / 15);
    public static final double kElbowGearRatio = (14 / 70) * (24 / 15);
    public static final double kShoulderEncoderDistancePerRotation = (kUpperArmLength * Math.PI) / kShoulderGearRatio;

    public static final double kElbowEncoderDistancePerRotation = (kForeArmLength * Math.PI) / kElbowGearRatio;

    public static final DCMotor kShoulderMotor = DCMotor.getNEO(1).withReduction(kShoulderGearRatio);
    public static final DCMotor kElbowMotor = DCMotor.getNEO(1).withReduction(kElbowGearRatio);
    // gearbox with 90% efficiency.
    public static final double kShoulderMaxAttainableSpeed = kShoulderMotor.freeSpeedRadPerSec * kUpperArmLength * 0.90;
    public static final double kElbowMaxAttainableSpeed = kElbowMotor.freeSpeedRadPerSec * kForeArmLength * 0.90;
    // time to top speed: 1.5 seconds.
    public static final double kShoulderMaxAcceleration = kShoulderMaxAttainableSpeed / 1.5;
    public static final double kElbowMaxAcceleration = kElbowMaxAttainableSpeed / 1.5;
    public static final double kAllowedErr = 0.1;

    public static final double kElbowTuckAngle = 0.0;
    public static final double kShoulderTuckMinimum = 0.0;
    public static final double kShoulderTuckMaximum = 135.0; // degrees.
    public static final double kHandoffShoulderAngle = 45.0; // degrees.
    public static final double kHandoffElbowAngle = 45.0; // degrees.
    public static final double kSourceShoulderAngle = 45.0; // degrees.
    public static final double kSourceElbowAngle = 45.0; // degrees.
    public static final double kAmpShoulderAngle = 45.0; // degrees.
    public static final double kAmpElbowAngle = 45.0; // degrees.
    public static final double kClimbShoulderAngle = 45.0; // degrees.
    public static final double kClimbElbowAngle = 45.0; // degrees.

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.
    public static final ArmFeedforward kShoulderFeedforward = new ArmFeedforward(1, 0.8, 0.15, 0.0);

    public static final ArmFeedforward kElbowFeedforward = new ArmFeedforward(1, 0.8, 0.15, 0.0);

    // Example value only - as above, this must be tuned for your drive!
    public static final Gains kPIDShoulder = new Gains(0.0, 0.0, 0.0, 0.0, 0.0);
    public static final Gains kPIDElbow = new Gains(0.0, 0.0, 0.0, 0.0, 0.0);
  }

  public static final class VisionConstants {
    public static final String kBackCameraName = "";
    public static final String kFrontCameraName ="";

    public static final int kFrontAprilTagPipeline = 1;
    public static final int kFrontNNPipeline = 2;

    public static final Transform3d kBackCameraToRobot = new Transform3d();
    public static final Transform3d kFrontCameraToRobot = new Transform3d();
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
