package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Basic simulation of a swerve module, will just hold its current state and not
 * use any hardware
 */
class SimSwerveModule {
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModulePosition getPosition() {
        return currentPosition;
    }

    public SwerveModuleState getState() {
        return currentState;
    }

    public void setDesiredState(SwerveModuleState targetState) {
        // Optimize the state
        currentState = SwerveModuleState.optimize(targetState, currentState.angle);

        currentPosition = new SwerveModulePosition(currentPosition.distanceMeters
                + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }
}