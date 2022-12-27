package frc.team7153.SwerveDrive.WheelTypes;

import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team7153.SwerveDrive.SwerveWheel;

public class SwerveWheel_Sim implements SwerveWheel {
    // States
    private double currentAngle;
    private double currentSpeed;
    private int updates = 0;

    // Config
    private boolean configDriveInvert = false;
    private boolean configSpinInvert = false;
    private double configAngleAdjust = 0.0;

    // Position
    private Translation2d pos;

    // Shuffleboard
    private NetworkTableEntry shuffle_speed;
    private NetworkTableEntry shuffle_angle;
    private NetworkTableEntry shuffle_updates;
    private NetworkTableEntry shuffle_driveInvert;
    private NetworkTableEntry shuffle_angleInvert;
    private NetworkTableEntry shuffle_angleAdjust;

    // Constructor
    /**
     * Creates a new simulated serve wheel. 
     * @param shuffleboardTabName Name of tab to output to in Shuffleboard
     * @param shuffleboardColumn Column in Shuffleboard to use (should be different for each wheel, starting with 1)
     * @param x The x position of the wheel
     * @param y The y position of the wheel
     */
    public SwerveWheel_Sim(String shuffleboardTabName, int shuffleboardColumn, double x, double y) {
        pos = new Translation2d(y, -x);

        ShuffleboardTab tab = Shuffleboard.getTab(shuffleboardTabName);
        ShuffleboardLayout column = tab.getLayout(String.format("Wheel %s", shuffleboardColumn), BuiltInLayouts.kList)
            .withPosition(shuffleboardColumn, 0)
            .withSize(1, 4)
            .withProperties(Map.of("Label position", "TOP"));

        shuffle_speed = column.add("Speed", 0.0)
            .getEntry();
        
        shuffle_angle = column.add("Angle", 0.0)
            .getEntry();
        
        shuffle_updates = column.add("Number of Updates", 0)
            .getEntry();
        
        shuffle_driveInvert = column.add("Drive Inverted", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();
        
        shuffle_angleInvert = column.add("Angle Inverted", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();
        
        shuffle_angleAdjust = column.add("Angle Adjust", 0.0)
            .getEntry();
        
        periodic();
    }

    // Config
    @Override
    public void config(boolean driveInverted, boolean spinInverted, double angleAdjust) {
        configDriveInvert = driveInverted;
        configSpinInvert = spinInverted;
        configAngleAdjust = angleAdjust;
    }

    // Position
    @Override
    public Translation2d getPosition() { return pos; }

    // Set
    @Override
    public void setAngle(double angle) { currentAngle = angle; }

    @Override
    public void setSpeed(double speed) { currentSpeed = speed; }

    @Override
    public void set(SwerveModuleState state) {
        set(state.angle.getDegrees(), state.speedMetersPerSecond);
    }

    // Periodic
    @Override
    public void periodic() {
        updates ++;

        shuffle_speed.setDouble(currentSpeed);
        shuffle_angle.setDouble(currentAngle);
        shuffle_updates.setNumber(updates);
        shuffle_driveInvert.setBoolean(configDriveInvert);
        shuffle_angleInvert.setBoolean(configSpinInvert);
        shuffle_angleAdjust.setDouble(configAngleAdjust);
    }
    
}
