package frc.robot.subsystems.SwerveModule;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        double angle;
        double speed;
    }

    public void updateInputs(SwerveModuleIOInputs inputs);
    public void stop();
    public void setState(SwerveModuleState state);
    public String getName();
    public SwerveModuleState getState();
    public SwerveModulePosition getPosition();
}
