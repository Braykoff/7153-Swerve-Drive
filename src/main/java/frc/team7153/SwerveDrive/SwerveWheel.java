package frc.team7153.SwerveDrive;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveWheel {
    // Motors, Encoders, PID
    private TalonFX driveWheel;
    private CANSparkMax spinWheel;

    private RelativeEncoder spinEncoder;

    private PIDController spinPID = new PIDController(0, 0, 0);

    // Position
    public Translation2d pos;

    // Config
    private double angleOffset = 0.0;

    // Targets
    private double targetSpeed = 0.0;

    /**
     * The max speed of the spin motor, from -1.0 to 1.0
     */
    public double maxSpinSpeed = 1.0;

    /**
     * The max velocity of the drive motor, RPMs
     */
    public double maxDriveSpeed = 100.0;

    /**
     * Creates a new Swerve Wheel. Expects a Falcon500 (TalonFX) for the drive wheel and a Rev Brushless NEO for spin motor.
     * Both should communicate over CAN bus.
     * 
     * @param drive The CAN id of the drive wheel (Falcon500)
     * @param spin The CAN id of the spin wheel (Rev Brushless NEO)
     * @param x The x position of the wheel, relative to the center of the base, in inches
     * @param y The y position of the wheel, relative to the center of the base, in inches
     */
    public SwerveWheel(int drive, int spin, double x, double y) {
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

    /**
     * Sets configuration for the wheel. This should be called as soon as the robot starts,
     * because it will not update already set speeds or angles.
     * @param driveInverted Whether the drive motor needs to be inverted (false by default)
     * @param spinInverted Whether the spin motor needs to be inverted (false by default)
     * @param angleAdjust A double added to each angle (0 by default), after it is inverted
     */
    public void config(boolean driveInverted, boolean spinInverted, double angleAdjust) {
        driveWheel.setInverted(driveInverted);
        spinWheel.setInverted(spinInverted);
        angleOffset = angleAdjust;
    }

    /**
     * Sets the angle of the wheel, from 0 - 360, with 0 degrees the front of the robot. 
     * @param angle
     */
    public void setAngle(double angle) {
        angle = SwerveMathUtils.normalizeAngle(angle + angleOffset);

        spinPID.setSetpoint(angle);
    }

    /**
     * Sets the speed of the wheel, in RPM
     * @param speed
     */
    public void setSpeed(double speed) {
        targetSpeed = speed;
    }

    /**
     * Sets the angle and speed of the wheel
     * @param angle degrees (0 - 360)
     * @param speed RPM
     */
    public void set(double angle, double speed) {
        setAngle(angle);
        setSpeed(speed);
    }

    /**
     * Set the angle and speed of the wheel with a SwerveModuleState
     * @param state
     */
    public void set(SwerveModuleState state) {
        SwerveModuleState.optimize(
            state, 
            new Rotation2d(SwerveMathUtils.degreesToRadians(SwerveMathUtils.normalizeAngle(spinEncoder.getPosition()*360)))
        );
        set(state.angle.getDegrees(), state.speedMetersPerSecond); // TODO MPS to RPM
    }

    /**
     * Needs to be called periodically to update PID loops
     */
    public void periodic() {
        spinWheel.set(
            SwerveMathUtils.symmetricClamp(
                spinPID.calculate(SwerveMathUtils.normalizeAngle(spinEncoder.getPosition()*360)), 
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