package frc.team7153.SwerveDrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveWheel {
    // Configs //

    /**
     * Sets configuration for the wheel. This should be called as soon as the robot starts,
     * because it will not update already set speeds or angles.
     * @param driveInverted Whether the drive motor needs to be inverted (false by default)
     * @param spinInverted Whether the spin motor needs to be inverted (false by default)
     * @param angleAdjust A double added to each angle (0 by default), after it is inverted
     */
    public void config(boolean driveInverted, boolean spinInverted, double angleAdjust);

    /**
     * Gets the position of the wheel, relative to the center of the robot
     * @return The position, in meters
     */
    public Translation2d getPosition();

    /**
     * Sets the max speeds for the drive motor and the spin motor
     * @param driveSpeed Max speed of the drive motor
     * @param spinSpeed Max speed of the spin motor 
     */
    public void setMaxSpeeds(double driveSpeed, double spinSpeed);

    // Driving //

    /**
     * Sets the angle of the wheel, from 0 - 360, with 0 degrees the front of the robot. 
     * @param angle
     */
    public void setAngle(double angle);

    /**
     * Sets the speed of the wheel, in MPS
     * @param speed
     */
    public void setSpeed(double speed);

    /**
     * Sets the angle and speed of the wheel
     * @param angle degrees (0 - 360)
     * @param speed Meters per second
     */
    default public void set(double angle, double speed) {
        setAngle(angle);
        setSpeed(speed);
    };

    /**
     * Set the angle and speed of the wheel with a SwerveModuleState
     * @param state
     */
    public void set(SwerveModuleState state);

    // Periodic //

    /**
     * Needs to be called periodically to update PID loops
     */
    public void periodic();
}