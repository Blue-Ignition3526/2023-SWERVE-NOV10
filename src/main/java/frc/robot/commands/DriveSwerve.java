// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Physical;
import frc.robot.subsystems.SwerveDrive;

public class DriveSwerve extends CommandBase {
  SwerveDrive m_swerveDrive;

  Supplier<Double> x;
  Supplier<Double> y;
  Supplier<Double> rot;

  SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.Swerve.Physical.kTeleopMaxAccelerationUnitsPerSecond);
  SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.Swerve.Physical.kTeleopMaxAccelerationUnitsPerSecond);
  SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.Swerve.Physical.kTeleopMaxAngularAccelerationUnitsPerSecond);
  
  public DriveSwerve(SwerveDrive swerveDrive, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot) {
    this.m_swerveDrive = swerveDrive;
    this.x = x;
    this.y = y;
    this.rot = rot;
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = this.x.get();
    double ySpeed = this.y.get();
    double rotSpeed = this.rot.get();

    xSpeed = Math.abs(xSpeed) > Constants.Operator.kDeadzone ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.Operator.kDeadzone ? ySpeed : 0.0;
    rotSpeed = Math.abs(rotSpeed) > Constants.Operator.kDeadzone ? rotSpeed : 0.0;

    xSpeed = xLimiter.calculate(xSpeed) * Physical.kTeleopMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * Physical.kTeleopMaxSpeedMetersPerSecond;
    rotSpeed = rotLimiter.calculate(rotSpeed) * Physical.kTeleopMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, m_swerveDrive.getRotation2d());
    SwerveModuleState[] m_moduleStates = m_swerveDrive.m_swerveDriveKinematics.toSwerveModuleStates(m_chassisSpeeds);
    m_swerveDrive.setModuleStates(m_moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
