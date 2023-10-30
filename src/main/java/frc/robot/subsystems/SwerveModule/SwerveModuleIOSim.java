package frc.robot.subsystems.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleIOSim implements SwerveModuleIO {
    // Swerve module name
    private final String m_name;

    // Current state
    private SwerveModuleState state = new SwerveModuleState();

    public SwerveModuleIOSim(Object[] Arr) {
        // Set module name
        m_name = (String) Arr[7];
    }

    public SwerveModuleIOSim(String name) {
        // Set module name
        m_name = name;
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.angle = state.angle.getRadians();
        inputs.speed = state.speedMetersPerSecond;
    }

    @Override
    public void stop() {
        state = new SwerveModuleState(0, state.angle);
    }

    @Override
    public void setState(SwerveModuleState state) {
        this.state = state;
    }

    @Override
    public String getName() {
        return this.m_name;
    }

    // Returns the current state
    public SwerveModuleState getState() {
        return state;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(0, new Rotation2d());
    }
}
