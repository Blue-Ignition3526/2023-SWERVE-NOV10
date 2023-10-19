// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
  // Create all swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(Constants.Swerve.Motors.kFrontLeftVars);
  private final SwerveModule m_frontRight = new SwerveModule(Constants.Swerve.Motors.kFrontRightVars);
  private final SwerveModule m_backLeft = new SwerveModule(Constants.Swerve.Motors.kBackLeftVars);
  private final SwerveModule m_backRight = new SwerveModule(Constants.Swerve.Motors.kBackRightVars);

  // Create a NavX gyro over I2C MXP
  private final AHRS m_gyro = new AHRS(I2C.Port.kMXP);

  public final SwerveDriveKinematics m_swerveDriveKinematics = new SwerveDriveKinematics( // Kinematics
    Constants.Swerve.Physical.m_frontLeftLocation,
    Constants.Swerve.Physical.m_frontRightLocation,
    Constants.Swerve.Physical.m_backLeftLocation,
    Constants.Swerve.Physical.m_backRightLocation
  );

  public SwerveDrive() {
    // Reset the gyro to 0 degrees
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        m_gyro.reset();
      } catch (Exception e) {
      }
    }).start();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) { 
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.Physical.kMaxSpeedMetersPerSecond);
    m_frontLeft.setState(desiredStates[0]);
    m_frontRight.setState(desiredStates[1]);
    m_backLeft.setState(desiredStates[2]);
    m_backRight.setState(desiredStates[3]);
  }

  public void stopModules() { 
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Math.toRadians(m_gyro.getAngle()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("SwerveDrive/TurningPIDController", Constants.Swerve.Module.getTurningPIDController());
    SmartDashboard.putNumber("SwerveDrive/RobotRotationDeg", m_gyro.getAngle());
  }
}
