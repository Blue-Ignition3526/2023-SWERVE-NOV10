// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final SwerveDrive m_swerveDrive = new SwerveDrive();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_swerveDrive.setDefaultCommand(
      new DriveSwerve(
        m_swerveDrive,
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getRightX()
      )
    );

    m_driverController.rightTrigger(0.1).whileTrue(new DriveSwerve(m_swerveDrive, () -> 0.0, () -> m_driverController.getRightTriggerAxis(), () -> 0.0));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
