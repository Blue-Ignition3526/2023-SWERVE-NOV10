package frc.robot.subsystems.SwerveDrive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule.SwerveModule;

public class SwerveDriveIOReal implements SwerveDriveIO {
    // Create all swerve modules
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;

    // Create a NavX gyro over I2C MXP
    private final AHRS m_gyro = new AHRS(I2C.Port.kMXP);

    // Create a swerve drive odometry instance to calculate robot position
    public final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Constants.Swerve.Physical.m_swerveDriveKinematics, getRotation2d(), new SwerveModulePosition[]{});

    // Store module states
    private SwerveModuleState[] states = {};

    public SwerveDriveIOReal(
        SwerveModule m_frontLeft,
        SwerveModule m_frontRight,
        SwerveModule m_backLeft,
        SwerveModule m_backRight
    ) {
        this.m_frontLeft = m_frontLeft;
        this.m_frontRight = m_frontRight;
        this.m_backLeft = m_backLeft;
        this.m_backRight = m_backRight;

        // Reset the gyro to 0 degrees
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                m_gyro.reset();
            } catch (Exception e) {}
        }).start();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        this.states = desiredStates;
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
        inputs.frontLeftAngle = states[0].angle.getRadians();
        inputs.frontLeftSpeed = states[0].speedMetersPerSecond;

        inputs.frontRightAngle = states[1].angle.getRadians();
        inputs.frontRightSpeed = states[1].speedMetersPerSecond;

        inputs.backLeftAngle = states[2].angle.getRadians();
        inputs.backLeftSpeed = states[2].speedMetersPerSecond;

        inputs.backRightAngle = states[3].angle.getRadians();
        inputs.backRightSpeed = states[3].speedMetersPerSecond;

        inputs.gyroAngle = getRotation2d().getDegrees();
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(this.m_gyro.getAngle());
    }
}
