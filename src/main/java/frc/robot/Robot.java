// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team7153.SwerveDrive.SwerveBase;
import frc.team7153.SwerveDrive.WheelTypes.*;

public class Robot extends TimedRobot {
  // Swerve Wheels (Actual)
  private SwerveWheel_FN fl = new SwerveWheel_FN(1, 2, -0.3, 0.4);
  private SwerveWheel_FN fr = new SwerveWheel_FN(3, 4, 0.3, 0.4);
  private SwerveWheel_FN rl = new SwerveWheel_FN(5, 6, -0.3, -0.4);
  private SwerveWheel_FN rr = new SwerveWheel_FN(7, 8, 0.3, -0.4);

  // Swerve Wheels (Testing)
  /*private SwerveWheel_Sim fl = new SwerveWheel_Sim("Swerve Sim", 0, -0.3, 0.4);
  private SwerveWheel_Sim fr = new SwerveWheel_Sim("Swerve Sim", 1, 0.3, 0.4);
  private SwerveWheel_Sim rl = new SwerveWheel_Sim("Swerve Sim", 2, -0.3, -0.4);
  private SwerveWheel_Sim rr = new SwerveWheel_Sim("Swerve Sim", 3, 0.3, -0.4);*/

  // Swerve Base
  private SwerveBase base = new SwerveBase(fl, fr, rl, rr);

  // Controller
  private Joystick joy1 = new Joystick(0);

  @Override
  public void robotInit() {
    base.setMaxSpeed(500.0, 0.7);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // OR:
    //base.periodic();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    base.drive(joy1.getY(), joy1.getX(), joy1.getTwist());
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
