// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SuperstructureManager;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final PowerDistribution m_pdh;

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain;
  private final Intake m_intake;
  private final SuperstructureManager m_superstructure;
  private final Vision m_vision;
  //private final LED m_led;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandStadiaController m_driverController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_pdh = new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    
    m_drivetrain = new Drivetrain();

    m_intake = new Intake();
    m_superstructure = new SuperstructureManager(new Arm(Optional.of(m_intake.getShoulderToChassisEncoder())), new Climber(), new Shooter(), m_intake);

    m_vision = new Vision(m_drivetrain, m_superstructure);
    //m_led = new LED(m_superstructure, m_vision);

    m_driverController =
      new CommandStadiaController(OperatorConstants.kDriverControllerPort);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.leftBumper().whileTrue(m_superstructure.sourceIntake(true).alongWith(m_vision.track(Vision.Goal.SOURCE)));
    m_driverController.rightBumper().whileTrue(m_superstructure.sourceIntake(false).alongWith(m_vision.track(Vision.Goal.SOURCE)));
    m_driverController.leftTrigger().whileTrue(m_superstructure.groundIntake(true).alongWith(m_vision.track(Vision.Goal.NOTE)));
    m_driverController.rightTrigger().whileTrue(m_superstructure.groundIntake(false).alongWith(m_vision.track(Vision.Goal.NOTE)));

    m_driverController.a().whileTrue(m_superstructure.ampShot(0).alongWith(m_vision.track(Vision.Goal.AMP)));
    m_driverController.b().whileTrue(m_superstructure.speakerShot(0, false).alongWith(m_vision.track(Vision.Goal.SPEAKER)));
    m_driverController.x().whileTrue(m_superstructure.speakerShot(0, true).alongWith(m_vision.track(Vision.Goal.SPEAKER)));

    m_driverController.y().toggleOnTrue(m_superstructure.climb(null).alongWith(m_vision.track(Vision.Goal.TRAP)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_superstructure);
  }
}
