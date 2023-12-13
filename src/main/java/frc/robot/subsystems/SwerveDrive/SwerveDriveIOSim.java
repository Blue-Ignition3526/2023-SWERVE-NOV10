package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule.SwerveModule;

public class SwerveDriveIOSim implements SwerveDriveIO {
    // Create all swerve modules
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;

    // Create a swerve drive odometry instance to calculate robot position
    private final SwerveDriveOdometry m_odometry;

    private ChassisSpeeds speeds = new ChassisSpeeds();
    private double heading = 0;
    private double speedsUpdated = Timer.getFPGATimestamp();

    public SwerveDriveIOSim(
        SwerveModule m_frontLeft,
        SwerveModule m_frontRight,
        SwerveModule m_backLeft,
        SwerveModule m_backRight
    ) {
        this.m_frontLeft = m_frontLeft;
        this.m_frontRight = m_frontRight;
        this.m_backLeft = m_backLeft;
        this.m_backRight = m_backRight;

        this.m_odometry = new SwerveDriveOdometry(Constants.Swerve.Physical.m_swerveDriveKinematics, getRotation2d(), new SwerveModulePosition[]{
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
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

    public void updateInputs(SwerveDriveIOInputs inputs) {
        inputs.frontLeftAngle = this.m_frontLeft.getState().angle.getRadians();
        inputs.frontLeftSpeed = this.m_frontLeft.getState().speedMetersPerSecond;

        inputs.frontRightAngle = this.m_frontRight.getState().angle.getRadians();
        inputs.frontRightSpeed = this.m_frontRight.getState().speedMetersPerSecond;

        inputs.backLeftAngle = this.m_backLeft.getState().angle.getRadians();
        inputs.backLeftSpeed = this.m_backLeft.getState().speedMetersPerSecond;

        inputs.backRightAngle = this.m_backRight.getState().angle.getRadians();
        inputs.backRightSpeed = this.m_backRight.getState().speedMetersPerSecond;

        inputs.gyroAngle = getRotation2d().getDegrees();

        inputs.xSpeed = speeds.vxMetersPerSecond;
        inputs.ySpeed = speeds.vyMetersPerSecond;
        inputs.rotSpeed = speeds.omegaRadiansPerSecond;

        this.heading += speeds.omegaRadiansPerSecond * (Timer.getFPGATimestamp() - this.speedsUpdated);
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        this.speeds = speeds;
        this.speedsUpdated = Timer.getFPGATimestamp();
    }

    public ChassisSpeeds getSpeeds() {
        return this.speeds;
    }

    public Rotation2d getRotation2d() {
        // Calculate the new robot heading angle using the angle theta provided 
        return new Rotation2d(this.heading);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
    }

    public SwerveDriveOdometry getOdometry() {
        try {
            m_odometry.update(getRotation2d(), new SwerveModulePosition[]{
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            });
        } catch (Exception e) {
            System.out.println("Error updating odometry: " + e);
        }
        return m_odometry;
    }
}
