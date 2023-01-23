// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;



public class Robot extends TimedRobot {
  // Components
  private CANCoder absEncoder = new CANCoder(11);
  private CANSparkMax spin = new CANSparkMax(7, MotorType.kBrushless);
  private RelativeEncoder relEncoder = spin.getEncoder();

  // Config
  private double absEncStart;

  // Shuffle
  private GenericEntry s1A, s1R, s2A, s2R, s3A, s3R; 

  private static final double G_RATIO = -150.0 / 7.0;
  private double ABS_ENCODER_CORRECTION = 178.77;
  private double relEncoderOffset =0; // in degrees of drive rotation

  // Config Abs Encoder
  @Override
  public void robotInit() {
    absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    ShuffleboardTab tab = Shuffleboard.getTab("Swerve Raw");

    s1A = tab.add("1 Absolute raw", 0.0).getEntry(); 
    s1R = tab.add("1 Relative Raw", 0.0).getEntry();

    s2A = tab.add("2 Abs WA", 0.0).getEntry();
    s2R = tab.add("2 Rel WA", 0.0).getEntry();

    s3A = tab.add("3 Abs WA Bounded ", 0.0).getEntry();
    s3R = tab.add("3 Rel WA Bounded", 0.0).getEntry();
  }

  public double getDriveAngleFromAbsolute () {
      return ABS_ENCODER_CORRECTION - absEncoder.getAbsolutePosition() ;
  }

  public double getDriveAngleFromRelative() { 
    double rv =  360.0 / G_RATIO * -1 *  relEncoder.getPosition();
    return rv - relEncoderOffset; 
  }
  // Run
  @Override
  public void teleopInit() {
    spin.set(0.0);

   //absEncStart = absEncoder.getAbsolutePosition();
   //relEncoder.setPosition((absEncoder.getAbsolutePosition() - 178.77) / 360.0 * (-150.0/7.0));

   double a = getDriveAngleFromAbsolute();
   double r = getDriveAngleFromRelative();
   relEncoderOffset = r - a;
  


   //spin.set(0.1);
  }
  
  // Output
  @Override
  public void disabledInit() {
    DriverStation.reportWarning(
      String.format(
        "Relative encoder went %s degrees, absolute encoder went %s degrees (started at %s degrees)", 
        relEncoder.getPosition()*360.0, 
        absEncoder.getAbsolutePosition(), 
        absEncStart),
      false);
  }

  private double normalizeAngle (double a) {
    a =  a - (360.0 * Math.floor (a/360.0));
    if (a > 180) {a -= 360.0;} 
     return a ;

  }

  @Override
  public void robotPeriodic() {
    // double absPosReport = absEncoder.getAbsolutePosition()-178.77;
    // double relPosReport =  -1*relEncoder.getPosition() * 360.0 * (-7.0/150.0);

    s1A.setDouble(absEncoder.getAbsolutePosition());
    s1R.setDouble(relEncoder.getPosition());

    s2A.setDouble(  getDriveAngleFromAbsolute() );
    s2R.setDouble(  getDriveAngleFromRelative());

    s3A.setDouble( normalizeAngle(getDriveAngleFromAbsolute()) );
    s3R.setDouble( normalizeAngle(getDriveAngleFromRelative()));
  }
}
