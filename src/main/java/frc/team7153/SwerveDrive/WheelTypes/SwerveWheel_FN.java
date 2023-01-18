package frc.team7153.SwerveDrive.WheelTypes;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.team7153.SwerveDrive.SwerveMathUtils;
import frc.team7153.SwerveDrive.SwerveWheel;

/**
 * Swerve Wheel that uses a Falcon500 for the drive motor and Neo Brushless (With CAN Spark Max) for spin motor
 */
public class SwerveWheel_FN implements SwerveWheel {
    // Motors, Encoders, PID
    private TalonFX driveWheel;
    private CANSparkMax spinWheel;

    private RelativeEncoder spinEncoder;

    private PIDController spinPID = new PIDController(0, 0, 0);

    // Position
    private Translation2d pos;

    @Override
    public Translation2d getPosition() { return pos; }

    // Config
    private double angleOffset = 0.0;

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
     * @param x The x position of the wheel, relative to the center of the base, in meters
     * @param y The y position of the wheel, relative to the center of the base, in meters
     */
    public SwerveWheel_FN(int drive, int spin, double x, double y) {
        driveWheel = new TalonFX(drive);
        spinWheel = new CANSparkMax(spin, MotorType.kBrushless);

        spinEncoder = spinWheel.getEncoder();

        /*
         * The X and Y values are implemented in WPI's library oddly:
         * "Positive x values represent moving toward the front of the robot whereas positive 
         * y values represent moving toward the left of the robot."
         * It is changed below to make it easier to understand.
         * 
         * See https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html 
         */
        pos = new Translation2d(y, -x);

        spinPID.enableContinuousInput(0.0, 360.0);
    }

    // Config
    @Override
    public void config(boolean driveInverted, boolean spinInverted, double angleAdjust) {
        driveWheel.setInverted(driveInverted);
        spinWheel.setInverted(spinInverted);
        angleOffset = angleAdjust;
    }

    // Get Angle from Encoder
    private double getAngle() {
        return SwerveMathUtils.normalizeAngle(spinEncoder.getPosition() * kDEGREES_PER_ROTATION);
    }

    // Set Speeds
    @Override
    public void setAngle(double angle) {
        angle = SwerveMathUtils.normalizeAngle(angle + angleOffset);

        spinPID.setSetpoint(angle);
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