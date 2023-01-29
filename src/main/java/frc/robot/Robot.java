// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import com.frc7153.SwerveDrive.SwerveBase;
import com.frc7153.SwerveDrive.WheelTypes.*;

public class Robot extends TimedRobot {
  // Swerve Wheels (spin, drive, abs encoder, posx, posy, absHomeLoc)
  // Height: 30.5 in (0.77 m), width: 20 in (0.51 m)
  private SwerveWheel_FN fl = new SwerveWheel_FN(8, 4, 12, -0.255, 0.385, 181.67);
  private SwerveWheel_FN fr = new SwerveWheel_FN(7, 3, 11, 0.255, 0.385, 178.77);
  private SwerveWheel_FN rl = new SwerveWheel_FN(10, 6, 14, -0.255, -0.385 , 8.35);
  private SwerveWheel_FN rr = new SwerveWheel_FN(9, 5, 13, 0.255, -0.385, 17.139);

  private SwerveBase base = new SwerveBase(fl, fr, rl, rr);

  // Controller
  private Joystick joy1 = new Joystick(0);
  private XboxController xbox1 = new XboxController(1);

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

  private double sp = 0.0;

  @Override
  public void teleopInit() {
    base.setAngle(0.0);
    sp = 0.0;
  }

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
    //base.setAngle(joy1.getThrottle() * 180);

    /*if (joy1.getRawButtonPressed(11)) {
      sp += 45.0;
      base.setAngle(sp);
      System.out.println(String.format("Angle is %s", sp));
    } else if (joy1.getRawButtonPressed(12)) {
      sp -= 45.0;
      base.setAngle(sp);
      System.out.println(String.format("Angle is %s", sp));
    }

    base.setSpeed(joy1.getThrottle() * 4.0);*/
    base.drive(xbox1.getLeftY(), -xbox1.getLeftX(), 0.0);
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
