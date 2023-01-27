package com.frc7153.SwerveDrive.WheelTypes;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.frc7153.SwerveDrive.SwerveMathUtils;

/**
 * Swerve Wheel that uses a Falcon500 for the drive motor and Neo Brushless (with CAN Spark Max) for spin motor.
 * Uses CANCoder absolute encoder for absolute position.
 */
public class SwerveWheel_FN2 implements SwerveWheel {
    private static final double k_SPIN_RATIO = -150.0/ 7.0;
    
    // Motors, Encoders, PID
    private TalonFX driveWheel;
    private CANSparkMax spinWheel;

    private RelativeEncoder spinRelEncoder;
    private CANCoder spinAbsEncoder;
    private SparkMaxPIDController spinPID;

    // PID Coefficients
    private static double spin_kP = 0.3; // 0.012013
    private static double spin_kI = 0.0; // 0.0
    private static double spin_kD = 0.0; // 0.00013784

    // Position
    private Translation2d pos;

    @Override
    public Translation2d getPosition() { return pos; }

    // Config
    private double REL_ENCODER_OFFSET = 0.0;

    // Shuffleboard
    private GenericEntry shuffle_abs;
    private GenericEntry shuffle_rel;

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
    public SwerveWheel_FN2(int spin, int drive, int canCoder, double x, double y, double spinHomeLocation) {
        // Establish and Configure Objects
        driveWheel = new TalonFX(drive);
        spinWheel = new CANSparkMax(spin, MotorType.kBrushless);

        spinAbsEncoder = new CANCoder(canCoder);
        spinRelEncoder = spinWheel.getEncoder();
        
        spinWheel.setSmartCurrentLimit(40);
        spinAbsEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        //spinAbsEncoder.configFeedbackCoefficient(-0.087890625, "deg", SensorTimeBase.PerSecond);

        // Determine Relative Encoder Offset
        double relEncPos = 360.0 / k_SPIN_RATIO * -1.0 * spinRelEncoder.getPosition();
        REL_ENCODER_OFFSET = relEncPos - (spinHomeLocation - spinAbsEncoder.getAbsolutePosition());


        spinRelEncoder.setPosition((spinAbsEncoder.getAbsolutePosition() - spinHomeLocation) * k_SPIN_RATIO / 360.0);

        // PID
        spinPID = spinWheel.getPIDController();

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
        ShuffleboardTab tab = Shuffleboard.getTab("Swerve Drive");

        shuffle_abs = tab.add("Output String (swervez)", "?")
            .getEntry();
        
        shuffle_rel = tab.add(String.format("Relative Encoder (%s)", spin), 0.0)
            .getEntry();
    }

    // Get Angle from Relative Encoder (degrees)
    private double getAngleFromRelative() {
        //double a = (360.0 / k_SPIN_RATIO * -1.0 * spinRelEncoder.getPosition()) - REL_ENCODER_OFFSET;
        //return SwerveMathUtils.normalizeAngle(a);
        return SwerveMathUtils.normalizeAngle(spinRelEncoder.getPosition() * -360.0 / k_SPIN_RATIO);
    }

    // Set Speeds
    @Override
    public void setAngle(double angle) {
        angle = SwerveMathUtils.normalizeAngle(angle);
        double _a = angle;
        //angle = (360.0 * k_SPIN_RATIO / -1.0 / angle) - REL_ENCODER_OFFSET;
        //angle = (-360.0 * angle * k_SPIN_RATIO) + REL_ENCODER_OFFSET;
        //angle = (angle / 360.0) * k_SPIN_RATIO
        angle = (angle / -360.0 * k_SPIN_RATIO); // Now in position of NEO
        angle = SwerveMathUtils.calculateContinuousMovement(spinRelEncoder.getPosition(), angle, 0 - k_SPIN_RATIO);
        //angle = angle - (angle % spinRelEncoder.getPosition()); // Nearest multiple
        //angle = Math.round(spinRelEncoder.getPosition() / angle) * angle;
        //System.out.println(String.format("Set point is %s, current pos is %s, target pos is %s", _a, spinRelEncoder.getPosition(), angle));
        //shuffleboardUpdate();
        spinPID.setReference(angle, ControlType.kPosition, 0); // this may be wrong
    }

    @Override
    public void setSpeed(double speed) {
        
    }

    @Override
    public void set(SwerveModuleState state) {
        SwerveModuleState.optimize(
            state, 
            Rotation2d.fromDegrees(getAngleFromRelative())
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
        //shuffle_abs.setDouble(spinAbsEncoder.getAbsolutePosition());
        shuffle_rel.setDouble(getAngleFromRelative());
    }
}