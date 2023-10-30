package frc.robot.subsystems.SwerveDrive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveDriveIO {
    @AutoLog
    public class SwerveDriveIOInputs {
        double frontLeftAngle;
        double frontLeftSpeed;

        double frontRightAngle;
        double frontRightSpeed;

        double backLeftAngle;
        double backLeftSpeed;

        double backRightAngle;
        double backRightSpeed;

        double gyroAngle;
    }

    public default void updateInputs(SwerveDriveIOInputs inputs) {};
    public default void stopModules() {};
    public default void setModuleStates(SwerveModuleState[] states) {};
    public SwerveModuleState[] getModuleStates();
    public Rotation2d getRotation2d();
    public SwerveDriveOdometry getOdometry();
}
