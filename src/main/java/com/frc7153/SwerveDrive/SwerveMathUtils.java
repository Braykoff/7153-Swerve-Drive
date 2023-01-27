package com.frc7153.SwerveDrive;

public class SwerveMathUtils {
    /**
     * Takes an angle and normalizes it to a range of -180 to 180
     * @param angle
     * @return The angle, normalized
     */
    public static double normalizeAngle(double angle) {
        angle = angle - (360.0 * Math.floor(angle / 360.0));
        if (angle > 180) { angle -= 360.0; }
        return angle;
    }

    /**
     * Determines how many times you have to add {@code gearRatio} to {@code setPoint} to get it as close
     * as possible to {@code currentPos} to ensure motor moves to least amount possible.<br><br>
     * Ex: If set-point is 5, gearing ratio is 360 (degrees), and it is currently at 4000, it will return 3965, 
     * which is equivalent to 5 (because degrees in a circle are continuos)
     * @param currentPos The current position
     * @param setPoint The target position
     * @param gearRatio Number of units needed to wrap all the way around (ie, degrees is 360, gears would be the gear ratio)
     * @return Number to move to that is basically (not numerically) equivalent to the {@code setPoint} but near the {@code currentPos}
     */
    public static double calculateContinuousMovement(double currentPos, double setPoint, double gearRatio) {
        return Math.floor(currentPos / gearRatio) * gearRatio + setPoint;
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