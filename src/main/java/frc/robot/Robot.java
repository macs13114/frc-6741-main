// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  VictorSPX LeftB = new VictorSPX(1);
  VictorSPX LeftF = new VictorSPX(2);
  VictorSPX RightF = new VictorSPX(3);
  VictorSPX RightB = new VictorSPX(4);
  Encoder encL = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
  Encoder encR = new Encoder(3, 4, true, Encoder.EncodingType.k2X);

  Joystick driver = new Joystick(0);

  private double targetDistance = 10.0;
  private double distanceTolerance = 0.2;
  private double setpoint;
  
  @Override
  public void robotInit() 
  {
    LeftB.follow(LeftF);
    RightB.follow(RightF);
    LeftF.setInverted(false);
    LeftB.setInverted(false);
    RightF.setInverted(true);
    RightB.setInverted(true);

    encL.setDistancePerPulse(1.0/803.0);
    encR.setDistancePerPulse(1.0/803.0);
    
    encL.reset();
    encR.reset();
  }

  @Override
  public void robotPeriodic() 
  {
    SmartDashboard.putNumber("Enc Left", encL.getDistance());
    SmartDashboard.putNumber("Enc Right", encR.getDistance());
  }

  @Override
  public void autonomousInit() 
  {
    encL.reset();
    encR.reset();
  }

  @Override
  public void autonomousPeriodic() 
  {
    if(encL.getDistance() < 5)
    {

    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() 
  {
    if(driver.getRawButtonPressed(1))
    {
      setpoint = 10.0;
    }
    if(driver.getRawButtonPressed(2))
    {
      setpoint = 0.0;
    }

    if(driver.getRawButton(3))
    {
      encL.reset();
      encR.reset();
    }

    SmartDashboard.putNumber("Setpoint", setpoint);
    
    double dis1 = encL.getDistance();
    double dis2 = encR.getDistance();

    double error = setpoint - dis1;
    double out = error * 0.1;
    double error2 = setpoint - dis2;
    double out2 = error2 * 0.1;
    
    LeftF.set(ControlMode.PercentOutput, out);
    RightF.set(ControlMode.PercentOutput, out2);
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
