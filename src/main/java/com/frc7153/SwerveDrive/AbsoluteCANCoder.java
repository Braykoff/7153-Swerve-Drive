package com.frc7153.SwerveDrive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

/**
 * CTRE's CANCoder implemented with Rev Robotic's Absolute Encoder,
 * so it can be used as a PID feedback device for a CAN Spark Max
 */
public class AbsoluteCANCoder implements AbsoluteEncoder {
    private CANCoder encoder;
    private double inversionFactor = 1.0; // Multiplied by output, either 1.0 or -1.0

    /**
     * Creates a new AbsoluteCANCoder object
     * @param CANid The CAN id of the encoder
     * @param CANbus The name of the CAN bus
     */
    public AbsoluteCANCoder(int CANid, String CANbus) {
        encoder = new CANCoder(CANid, CANbus);

        encoder.configFeedbackCoefficient(0.087890625 / 360.0, "rotations", SensorTimeBase.PerSecond);
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    }

    /**
     * Creates a new AbsoluteCANCoder object
     * @param CANid The CAN id of the encoder
     */
    public AbsoluteCANCoder(int CANid) { this(CANid, "rio"); }

    // Phoenix Error to REVLibError
    private REVLibError phoenixErrorToRevError(ErrorCode err) {
        return (err.equals(ErrorCode.OK) ? REVLibError.kOk : REVLibError.kError);
    }

    @Override
    public double getPosition() {
        return encoder.getPosition() * inversionFactor;
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity() * inversionFactor;
    }

    @Override
    public REVLibError setPositionConversionFactor(double factor) {
        return phoenixErrorToRevError(
            encoder.configFeedbackCoefficient(factor, "units", SensorTimeBase.PerSecond)
        );
    }

    @Override
    public double getPositionConversionFactor() { return encoder.configGetFeedbackCoefficient(); }

    @Override
    public REVLibError setVelocityConversionFactor(double factor) {
        return phoenixErrorToRevError(
            encoder.configFeedbackCoefficient(factor, "units", SensorTimeBase.PerSecond);
        );
    }

    @Override
    public double getVelocityConversionFactor() { return encoder.configGetFeedbackCoefficient(); }

    @Override
    public REVLibError setInverted(boolean inverted) {
        inversionFactor = (inverted) ? -1.0 : 1.0;
        return REVLibError.kOk;
    }

    @Override
    public boolean getInverted() { return inversionFactor == -1.0; }

    @Override
    public REVLibError setAverageDepth(int depth) { return REVLibError.kNotImplemented; }

    @Override
    public int getAverageDepth() { return 0; }

    /**
     * Set the zero offset of the encoder in degrees.
     * @param offset The offset, in degrees
     */
    @Override
    public REVLibError setZeroOffset(double offset) {
        return phoenixErrorToRevError(
            encoder.configMagnetOffset(offset)
        );
    }

    @Override
    public double getZeroOffset() {
        // TODO Auto-generated method stub
        return 0;
    }

    
}