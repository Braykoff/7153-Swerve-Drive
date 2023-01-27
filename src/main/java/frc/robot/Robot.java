// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import com.frc7153.SwerveDrive.SwerveBase;
import com.frc7153.SwerveDrive.WheelTypes.*;

public class Robot extends TimedRobot {
  // Swerve Wheels (spin, drive, abs encoder, posx, posy, absHomeLoc)
  // Height: 30.5 in (0.77 m), width: 20 in (0.51 m)
  //private SwerveWheel_FN2 fl = new SwerveWheel_FN2(8, 4, 12, -0.255, 0.385, 180.088);
  private SwerveWheel_FN2 fr = new SwerveWheel_FN2(7, 3, 11, 0.255, 0.385, 178.77);
  //private SwerveWheel_FN2 rl = new SwerveWheel_FN2(10, 6, 14, -0.255, -0.385 , 8.35);
  //private SwerveWheel_FN2 rr = new SwerveWheel_FN2(9, 5, 13, 0.255, -0.385, 17.139);

  //private SwerveBase base = new SwerveBase(fl, fr, rl, rr);

  // Controller
  private Joystick joy1 = new Joystick(0);

  @Override
  public void robotInit() {
    //base.setMaxSpeed(500.0, 0.7);
  }

  @Override
  public void robotPeriodic() {
    //CommandScheduler.getInstance().run();
    // OR:
    //base.periodic();
  }

  @Override
  public void autonomousInit() {
    //base.setAngle(90.0);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {
    //base.setAngle(0.0);
    /*double sp = joy1.getThrottle();
    sp += 1.0;
    sp /= 2.0;
    sp *= 360.0;
    base.setAngle(sp);
    DriverStation.reportWarning(String.format("Setpoint is -> %s", sp), false);*/
    //fr.setAngle(90.0);
    fr.setAngle(joy1.getThrottle() * 180);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
