package com.frc7153.SwerveDrive;

public class SwerveMathUtils {
    /**
     * Takes an angle and normalizes it to a range of 0 - 360
     * @param angle
     * @return The angle, normalized
     */
    public static double normalizeAngle(double angle) {
        return angle % 360.0;
    }

    /**
     * Clamps a speed to the specified range, assuming the lower bound is the opposite of the upper bound
     * @param value The input value
     * @param clamp The upper bound, and opposite of the lower bound
     * @return The clamped value
     */
    public static double symmetricClamp(double value, double clamp) {
        if (clamp < 0) { clamp = -clamp;}
        return Math.min(Math.max(value, -clamp), clamp);
    }

    /**
     * Convert RPMs to Falcon500's encoder velocity
     * @param rpm
     * @return Falcon500's encoder velocity
     */
    public static double rpmToFalcon500Velocity(double rpm) { return rpm * 2048 * 0.1; }

    /**
     * Convert the Falcon500's velocity to RPMs
     * @param velocity
     * @return RPMs
     */
    public static double falcon500VelocityToRPM(double velocity) { return velocity / 2048 / 0.1; }

    /**
     * Convert degrees to radians
     * @param angles
     * @return radians
     */
    public static double degreesToRadians(double angles) { return angles * (Math.PI/180); }
}