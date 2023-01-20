package com.frc7153.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.frc7153.SwerveDrive.WheelTypes.SwerveWheel;

public class SwerveBase extends SubsystemBase {
    // Wheels
    private SwerveWheel fl;
    private SwerveWheel fr;
    private SwerveWheel rl;
    private SwerveWheel rr;
    
    // Kinematics
    private SwerveDriveKinematics kinematics;

    // State
    private boolean periodicRunning = true;

    // Max Speeds
    private double maxDriveSpeed = 100.0;
    private double maxSpinSpeed = 10.0;
    
    /**
     * Creates a new SwerveBase, with four SwerveWheels.
     * @param frontLeft
     * @param frontRight
     * @param rearLeft
     * @param rearRight
     */
    public SwerveBase(SwerveWheel frontLeft, SwerveWheel frontRight, SwerveWheel rearLeft, SwerveWheel rearRight) {
        fl = frontLeft;
        fr = frontRight;
        rl = rearLeft;
        rr = rearRight;

        kinematics = new SwerveDriveKinematics(fl.getPosition(), fr.getPosition(), rl.getPosition(), rr.getPosition());
    }

    /**
     * Sets the max speed for all the wheels. See the specific classes for units.
     * @param driveSpeed Max speed of driving (Meters/Second)
     * @param spinSpeed Max speed of spin (Degrees/Second)
     */
    public void setMaxSpeed(double driveSpeed, double spinSpeed) {
        maxDriveSpeed = driveSpeed;
        maxSpinSpeed = spinSpeed;
    }

    /**
     * Drives the robot
     * @param y forward/backward speed (meters per second)
     * @param x left/right speed (meters per second)
     * @param r rotation speed (degrees per second)
     */
    public void driveAbsolute(double y, double x, double r) {
        ChassisSpeeds speed = new ChassisSpeeds(y, x, -SwerveMathUtils.degreesToRadians(r));
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);

        fl.set(states[0]);
        fr.set(states[1]);
        rl.set(states[2]);
        rr.set(states[3]);
        periodicRunning = true;
    }

    /**
     * Drives the robot, using a percentage for speed
     * @param y forward/backward speed (percentage of max drive speed)
     * @param x left/right speed (percentage of max drive speed)
     * @param r rotation speed (percentage of max spin speed)
     */
    public void drive(double y, double x, double r) { driveAbsolute(y*maxDriveSpeed, x*maxDriveSpeed, r*maxSpinSpeed); }

    /**
     * Drives the robot, with field-oriented drive
     * @param y forward/backward speed (meters per second)
     * @param x left/right speed (meters per second)
     * @param r rotation speed (degrees per second)
     * @param deg current angle of the robot (degrees)
     */
    public void driveFieldOrientedAbsolute(double y, double x, double r, double deg) {
        ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(
            y, x, 
            -SwerveMathUtils.degreesToRadians(r), 
            Rotation2d.fromDegrees(deg)
        );
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);

        fl.set(states[0]);
        fr.set(states[1]);
        rl.set(states[2]);
        rr.set(states[3]);
        periodicRunning = true;
    }

    /**
     * Drives the robot, with field-oriented drive, using a percentage for speed
     * @param y forward/backward speed (percentage of max drive speed)
     * @param x left/right speed (percentage of max drive speed)
     * @param r rotation (percentage of max spin speed)
     * @param deg current angle of the robot (degrees)
     */
    public void driveFieldOriented(double y, double x, double r, double deg) { driveFieldOrientedAbsolute(y*maxDriveSpeed, x*maxDriveSpeed, r*maxSpinSpeed, deg); }

    /**
     * Orbits the robot around a point. There is no non-absolute implementation of this yet.
     * @param rotation The speed, in degrees per second
     * @param centerX The X position of the center of rotation, relative to the robot's base (meters)
     * @param centerY The Y position of the center of rotation, relative to the robot's base (meters)
     */
    public void orbitAbsolute(double rotation, double centerX, double centerY) {
        ChassisSpeeds speed = new ChassisSpeeds(0.0, 0.0, -SwerveMathUtils.degreesToRadians(rotation));
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed, new Translation2d(centerX, centerY));

        fl.set(states[0]);
        fr.set(states[1]);
        rl.set(states[2]);
        rr.set(states[3]);
        periodicRunning = true;
    }

    /**
     * Drives the robot like its a tank drive base
     * @param left The speed of the left side of the robot (meters/second)
     * @param right The speed of the right side of the robot (meters/second)
     */
    public void tankDriveAbsolute(double left, double right) {
        fl.set(0.0, left);
        rl.set(0.0, left);
        fr.set(0.0, right);
        rr.set(0.0, right);

        periodicRunning = true;
    }

    /**
     * Drives the robot like its a tank drive base, using percentages for speed
     * @param left The speed of the left side of the robot (percentage of max drive speed)
     * @param right The speed of the right side of the robot (percentage of max drive speed)
     */
    public void tankDrive(double left, double right) { tankDriveAbsolute(left*maxDriveSpeed, right*maxDriveSpeed); }

    /**
     * Stops the robot
     * @param reset Whether the wheels should return to a forward position
     */
    public void stop(boolean reset) {
        if (reset) {
            fl.set(0.0, 0.0);
            fr.set(0.0, 0.0);
            fr.set(0.0, 0.0);
            rr.set(0.0, 0.0);
        } else {
            fl.setSpeed(0.0);
            fr.setSpeed(0.0);
            fr.setSpeed(0.0);
            rr.setSpeed(0.0);
        }
    }

    /**
     * Stops the periodic methods.
     * Periodic methods will resume as soon as one of the drive methods are called again.
     * @param reset Whether the motor speeds should be set to 0 (if false, the robot may keep moving)
     */
    public void stopPeriodic(boolean reset) {
        if (reset) { stop(false); periodic(); }
        periodicRunning = false;
    }

    // Periodic
    @Override
    public void periodic() {
        if (!DriverStation.isDisabled() && periodicRunning) {
            fl.periodic();
            fr.periodic();
            rl.periodic();
            rr.periodic();
        }
    }
}
