package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  // Motors
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  // Motor encoder
  private final RelativeEncoder m_turningMotorEncoder;
  private final RelativeEncoder m_driveMotorEncoder;

  // Absolute encoder
  private final CANCoder m_turningAbsoluteEncoder; 
  private final double m_turningEncoderOffsetRad;
  private final boolean m_turningAbsoluteEncoderInverted;

  // Swerve module name
  private final String m_name;

  public SwerveModule(Object[] Arr) {
    // Get variables from options and add them to the class
    this.m_turningEncoderOffsetRad = (double)Arr[0]; 
    this.m_turningAbsoluteEncoderInverted = (boolean)Arr[1]; 
    this.m_turningAbsoluteEncoder = new CANCoder((int)Arr[2]); 

    this.m_driveMotor = new CANSparkMax((int)Arr[3], MotorType.kBrushless); 
    this.m_turningMotor = new CANSparkMax((int)Arr[4], MotorType.kBrushless);

    this.m_driveMotor.setInverted((boolean)Arr[5]); 
    this.m_turningMotor.setInverted((boolean)Arr[6]); 

    this.m_driveMotorEncoder = m_driveMotor.getEncoder(); 
    this.m_turningMotorEncoder = m_turningMotor.getEncoder();

    this.m_turningAbsoluteEncoder.configMagnetOffset(Math.toDegrees(m_turningEncoderOffsetRad) + Constants.Swerve.Module.kGlobalModuleTurningOffset);

    // Configure encoder conversions
    this.m_driveMotorEncoder.setPositionConversionFactor(Constants.Swerve.Module.kDriveEncoder_RotationToMeter); 
    this.m_driveMotorEncoder.setVelocityConversionFactor(Constants.Swerve.Module.kDriveEncoder_RPMToMeterPerSecond);
    this.m_turningMotorEncoder.setPositionConversionFactor(Constants.Swerve.Module.kTurningEncoder_RotationToRadian); 
    this.m_turningMotorEncoder.setVelocityConversionFactor(Constants.Swerve.Module.kTurningEncoder_RPMToRadianPerSecond);

    // PID Continuous input
    Constants.Swerve.Module.getTurningPIDController().enableContinuousInput(0, Math.PI * 2);

    // Set module name
    m_name = (String) Arr[7];

    // Reset the motor encoder with the value of the absolute encoder + the offset
    resetMotorEncoders();
  }

  double getAbsoluteEncoderRad() {
    // Return the absolute encoder position in degrees
    return Math.toRadians(m_turningAbsoluteEncoder.getAbsolutePosition()) * (m_turningAbsoluteEncoderInverted ? -1.0 : 1.0);
  }

  void resetMotorEncoders() {
    // Make sure it's using radians instead of rotations
    this.m_turningMotorEncoder.setPositionConversionFactor(Constants.Swerve.Module.kTurningEncoder_RotationToRadian); 
    this.m_turningMotorEncoder.setVelocityConversionFactor(Constants.Swerve.Module.kTurningEncoder_RPMToRadianPerSecond);

    // Reset the motor encoder with the value of the absolute encoder + the offset
    // The position of the encoder is given from [0, 360) degrees
    // however the getAbsoluteEncoderRad() function returns the value in radians and inverts it if needed
    // The offset is already in radians so it is just added
    m_turningMotorEncoder.setPosition(getAbsoluteEncoderRad());

    // Make sure it's using meters instead of rotations
    this.m_driveMotorEncoder.setPositionConversionFactor(Constants.Swerve.Module.kDriveEncoder_RotationToMeter); 
    this.m_driveMotorEncoder.setVelocityConversionFactor(Constants.Swerve.Module.kDriveEncoder_RPMToMeterPerSecond);

    // We reset the drive motor position to start measuring distance from 0
    m_driveMotorEncoder.setPosition(0);
  }

  double getTurningEncoderPositionRad() {
    // Make sure it's using radians instead of rotations
    this.m_turningMotorEncoder.setPositionConversionFactor(Constants.Swerve.Module.kTurningEncoder_RotationToRadian); 
    this.m_turningMotorEncoder.setVelocityConversionFactor(Constants.Swerve.Module.kTurningEncoder_RPMToRadianPerSecond);

    // Return the motor encoder position in radians
    return this.m_turningMotorEncoder.getPosition() % (2 * Math.PI);
  }

  void stop() {
    // Stop the motors (Set the speed to 0)
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  void setState(SwerveModuleState state) {
    // Check if the speed or rotation difference is meaningful
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
  }
    // Optimize the state
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, new Rotation2d(getTurningEncoderPositionRad()));

    // Set the drive motor speed
    // The speed is given in meters per second, so we need to convert from [-1, 1]
    // To do that we divide it by the max speed of robot
    m_driveMotor.set(optimizedState.speedMetersPerSecond / Constants.Swerve.Physical.kMaxSpeedMetersPerSecond);

    // Set the turning motor speed
    // The speed is not given to us, rather the angle we want to turn to
    // So we need to calculate the difference between the current angle and the target angle
    // Then we use the PID controller to calculate the speed we need to turn at
    m_turningMotor.set(Constants.Swerve.Module.getTurningPIDController().calculate(getTurningEncoderPositionRad(), optimizedState.angle.getRadians()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("SwerveDrive/" + m_name + "/DegTurningMotorEncoder", Math.toDegrees(getTurningEncoderPositionRad()));
    SmartDashboard.putNumber("SwerveDrive/" + m_name + "/DegAbsoluteEncoder", Math.toDegrees(getAbsoluteEncoderRad()));

    SmartDashboard.putNumber("SwerveDrive/" + m_name + "/RadTurningMotorEncoder", getTurningEncoderPositionRad());
    SmartDashboard.putNumber("SwerveDrive/" + m_name + "/RadAbsoluteEncoder", getAbsoluteEncoderRad());
  }
}
