// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.SwerveDriveIOReal;
import frc.robot.subsystems.SwerveDrive.SwerveDriveIOSim;
import frc.robot.subsystems.SwerveModule.SwerveModule;
import frc.robot.subsystems.SwerveModule.SwerveModuleIOSim;
import frc.robot.subsystems.SwerveModule.SwerveModuleIOSparkMax;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  SwerveModule m_frontLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backLeft;
  SwerveModule m_backRight;

  SwerveDrive m_swerveDrive;

  public RobotContainer() {
    if (Robot.isReal()) {
      this.m_frontLeft = new SwerveModule(new SwerveModuleIOSparkMax(Constants.Swerve.Motors.kFrontLeftVars));
      this.m_frontRight = new SwerveModule(new SwerveModuleIOSparkMax(Constants.Swerve.Motors.kFrontRightVars));
      this.m_backLeft = new SwerveModule(new SwerveModuleIOSparkMax(Constants.Swerve.Motors.kBackLeftVars));
      this.m_backRight = new SwerveModule(new SwerveModuleIOSparkMax(Constants.Swerve.Motors.kBackRightVars));
      
      this.m_swerveDrive = new SwerveDrive(new SwerveDriveIOReal(m_frontLeft, m_frontRight, m_backLeft, m_backRight));

      System.out.println("Real");
    } else {
      this.m_frontLeft = new SwerveModule(new SwerveModuleIOSim(Constants.Swerve.Motors.kFrontLeftVars));
      this.m_frontRight = new SwerveModule(new SwerveModuleIOSim(Constants.Swerve.Motors.kFrontRightVars));
      this.m_backLeft = new SwerveModule(new SwerveModuleIOSim(Constants.Swerve.Motors.kBackLeftVars));
      this.m_backRight = new SwerveModule(new SwerveModuleIOSim(Constants.Swerve.Motors.kBackRightVars));
      
      this.m_swerveDrive = new SwerveDrive(new SwerveDriveIOSim(m_frontLeft, m_frontRight, m_backLeft, m_backRight));

      System.out.println("Sim");
    }

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
