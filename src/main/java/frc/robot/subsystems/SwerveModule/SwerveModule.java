package frc.robot.subsystems.SwerveModule;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public SwerveModule(SwerveModuleIO io) {
        this.io = io;
    }

    public void stop() {
        io.stop();
    }

    public void setState(SwerveModuleState state) {
        io.setState(state);
    }

    public String getName() {
        return io.getName();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("SwerveDrive/SwerveModule/" + io.getName(), inputs);

        Logger.getInstance().recordOutput("SwerveDrive/SwerveModule/" + io.getName() + "/Speed", io.getState().speedMetersPerSecond);
        Logger.getInstance().recordOutput("SwerveDrive/SwerveModule/" + io.getName() + "/AngleSetpointRad", io.getState().angle.getRadians());
        Logger.getInstance().recordOutput("SwerveDrive/SwerveModule/" + io.getName() + "/AngleSetpointDeg", io.getState().angle.getDegrees());
        Logger.getInstance().recordOutput("SwerveDrive/SwerveModule/" + io.getName() + "/AngleRad", io.getPosition().angle.getRadians());
        Logger.getInstance().recordOutput("SwerveDrive/SwerveModule/" + io.getName() + "/AngleDeg", io.getPosition().angle.getDegrees());
    }

    public SwerveModuleState getState() {
        return io.getState();
    }

    public SwerveModulePosition getPosition() {
        return io.getPosition();
    }
}

