package com.frc7153.SwerveDrive.WheelTypes;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import com.frc7153.SwerveDrive.SwerveMathUtils;

/**
 * Swerve Wheel that uses a Falcon500 for the drive motor and Neo Brushless (with CAN Spark Max) for spin motor.
 * Uses CANCoder absolute encoder for absolute position.
 */
public class SwerveWheel_FN implements SwerveWheel {
    // Motors, Encoders, PID
    private TalonFX driveWheel;
    private CANSparkMax spinWheel;

    private RelativeEncoder spinRelEncoder;
    private SparkMaxPIDController spinPID;

    // Position
    private Translation2d pos;

    @Override
    public Translation2d getPosition() { return pos; }

    // Config
    private double relAngleOffset;

    // Targets
    private double targetSpeed = 0.0;

    // Speeds (Absolute fail-safe max speeds, not for speed control [use SwerveBase max speed instead])
    private double maxDriveSpeed = 100.0;
    private double maxSpinSpeed = 1.0;

    // Real-World Measurements
    private static double kMETERS_PER_ROTATION = 0.1; // Number of meters traveled for each rotation of the DRIVE motor
    private static double kDEGREES_PER_ROTATION = 360; // Number of degrees rotated for each rotation of the SPIN motor

    /**
     * Creates a new Swerve Wheel. Expects a Falcon500 (TalonFX) for the drive wheel and a Rev Brushless NEO for spin motor.
     * Both should communicate over CAN bus.
     * 
     * @param drive The CAN id of the drive wheel (Falcon500)
     * @param spin The CAN id of the spin wheel (Rev Brushless NEO)
     * @param canCoder The CAN id of the CANCoder
     * @param x The x position of the wheel, relative to the center of the base, in meters
     * @param y The y position of the wheel, relative to the center of the base, in meters
     * @param spinHomeLocation When the wheel's direction is 0 degrees (forward), what is the output of the ABSOLUTE encoder (degrees)?
     */
    public SwerveWheel_FN(int drive, int spin, int canCoder, double x, double y, double spinHomeLocation) {
        driveWheel = new TalonFX(drive);
        spinWheel = new CANSparkMax(spin, MotorType.kBrushless);

        CANCoder spinAbsEncoder = new CANCoder(canCoder);
        spinRelEncoder = spinWheel.getEncoder();

        relAngleOffset = SwerveMathUtils.normalizeAngle(spinHomeLocation - spinAbsEncoder.getPosition()); // TODO: this is wrong

        spinPID = spinWheel.getPIDController();

        //TODO: implement
        spinPID.setP(0.1);

        /*
         * The X and Y values are implemented in WPI's library oddly:
         * "Positive x values represent moving toward the front of the robot whereas positive 
         * y values represent moving toward the left of the robot."
         * It is changed below to make it easier to understand.
         * 
         * See https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html 
         */
        pos = new Translation2d(y, -x);
    }

    // Config
    @Override
    public void config(boolean driveInverted, boolean spinInverted) {
        driveWheel.setInverted(driveInverted);
        spinWheel.setInverted(spinInverted);

        spinRelEncoder.setInverted(spinInverted);
    }

    // Get Angle from Encoder (degrees)
    private double getAngle() {
        return SwerveMathUtils.normalizeAngle(spinRelEncoder.getPosition()*360 - relAngleOffset);
    }

    // Set Speeds
    @Override
    public void setAngle(double angle) {
        angle = SwerveMathUtils.normalizeAngle(angle - relAngleOffset);
        spinWheel.
    }

    @Override
    public void setSpeed(double speed) {
        targetSpeed = (speed*60)/kMETERS_PER_ROTATION;
    }

    @Override
    public void set(SwerveModuleState state) {
        SwerveModuleState.optimize(
            state, 
            Rotation2d.fromDegrees(getAngle())
        );
        set(state.angle.getDegrees(), state.speedMetersPerSecond);
    }

    // Periodic
    @Override
    public void periodic() {
        spinWheel.set(
            SwerveMathUtils.symmetricClamp(
                spinPID.calculate(getAngle()), 
                maxSpinSpeed
            )
        );

        driveWheel.set(
            TalonFXControlMode.Velocity,
            SwerveMathUtils.rpmToFalcon500Velocity(SwerveMathUtils.symmetricClamp(
                targetSpeed + spinEncoder.getVelocity(),
                maxDriveSpeed
            )
        ));
    }
}