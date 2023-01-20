package com.frc7153.SwerveDrive.WheelTypes;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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
    private CANCoder spinAbsEncoder;
    private SparkMaxPIDController spinPID;

    // PID Coefficients
    private static double spin_kP = 0.012013;
    private static double spin_kI = 0.0;
    private static double spin_kD = 0.00013784;

    // Position
    private Translation2d pos;

    @Override
    public Translation2d getPosition() { return pos; }

    // Config
    private double absAngleOffset;

    // Shuffleboard
    private GenericEntry shuffle_abs;
    private GenericEntry shuffle_rel;
    private GenericEntry shuffle_absPos;
    private GenericEntry shuffle_relPos;

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
    public SwerveWheel_FN(int spin, int drive, int canCoder, double x, double y, double spinHomeLocation) {
        driveWheel = new TalonFX(drive);
        spinWheel = new CANSparkMax(spin, MotorType.kBrushless);

        spinAbsEncoder = new CANCoder(canCoder);
        spinRelEncoder = spinWheel.getEncoder();
        spinRelEncoder.setPosition(0.0);

        absAngleOffset = (spinAbsEncoder.getAbsolutePosition() - spinHomeLocation);

        spinPID = spinWheel.getPIDController();

        // Config CAN Spark Max
        spinWheel.setSmartCurrentLimit(40);

        spinPID.setP(spin_kP, 0);
        spinPID.setI(spin_kI, 0);
        spinPID.setD(spin_kD, 0);
        spinPID.setOutputRange(-10.0, 10.0);

        /*
         * The X and Y values are implemented in WPI's library oddly:
         * "Positive x values represent moving toward the front of the robot whereas positive 
         * y values represent moving toward the left of the robot."
         * It is changed below to make it easier to understand.
         * 
         * See https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html 
         */
        pos = new Translation2d(y, -x);

        // Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab("Swerve Drive Output");

        shuffle_abs = tab.add(String.format("Absolute Encoder (%s)", canCoder), spinAbsEncoder.getAbsolutePosition())
            .getEntry();
        
        shuffle_rel = tab.add(String.format("Relative Encoder (%s)", spin), spinRelEncoder.getPosition())
            .getEntry();
        
        shuffle_relPos = tab.add(String.format("Rel Enc Pos (%s)", spin), -1.0)
            .getEntry();
        
        shuffle_absPos = tab.add(String.format("Abs Enc Pos (%s)", canCoder), -1.0)
            .getEntry();
        
        tab.add(String.format("Abs Angle Offset (%s)", spin), absAngleOffset);
    }

    // Config (Not implemented, will be done later) TODO
    @Override
    public void config(boolean driveInverted, boolean spinInverted) {
        driveWheel.setInverted(driveInverted);
        spinWheel.setInverted(spinInverted);

        spinRelEncoder.setInverted(spinInverted);
    }

    // Get Angle from Encoder (degrees)
    private double getAngle() {
        return (spinRelEncoder.getPosition() * 360.0 * (7.0/150.0) + absAngleOffset);
    }

    // Set Speeds
    @Override
    public void setAngle(double angle) {
        //angle = SwerveMathUtils.normalizeAngle(angle - absAngleOffset);
        angle = angle - absAngleOffset;
        angle = angle * (150.0/7.0) * (1.0/360.0);
        spinPID.setReference(angle, ControlType.kPosition, 0);
    }

    @Override
    public void setSpeed(double speed) {

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
    /*
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
    }*/
    public void periodic() {}

    // Shuffleboard
    @Override
    public void shuffleboardUpdate() {
        shuffle_abs.setDouble(spinAbsEncoder.getAbsolutePosition());
        shuffle_rel.setDouble(spinRelEncoder.getPosition());

        shuffle_absPos.setDouble(spinAbsEncoder.getAbsolutePosition() - absAngleOffset);
        shuffle_relPos.setDouble(getAngle());
    }
}